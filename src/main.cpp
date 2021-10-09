#include <Arduino.h>
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_wifi.h"
#include <Wire.h>
#include <WiFi.h>

#include "EasyBuzzer.h"
#include "Adafruit_SGP30.h"
#include "MS5611.h"

#include "SHT3x.h"

#include "PubSubClient.h"
#include "EEPROM.h"

#define SHT3X_ADDRESS 0x44
#define SGP30_ADDRESS 0x58
#define MS5611_ADDRESS 0X77

typedef struct
{
	u_short index = 0;
	float temperature = 0;
	float humidity = 0;
	uint16_t TVOC = 0;
	uint16_t eCO2 = 0;
	uint16_t eCO2_base = 0;
	uint16_t TVOC_base = 0;
	uint16_t rawH2 = 0;
	uint16_t rawEthanol = 0;
	int32_t pressure = 0;
	float ms5611Temperature = 0;
	unsigned long durationWiFi = 0;
	unsigned long durationMqtt = 0;
} SensorData;

// EEPROM配置
#define EEPROM_ADDRESS_DATASIZE 0				 // 缓存了多少组数据
#define EEPROM_LEN_DATASIZE sizeof(uint32_t)	 // 4
#define EEPROM_LEN_SENSORDATA sizeof(SensorData) // 36
#define DATASIZE_POST 5							 // 缓存数据达到多少次发送一次请求

// RTC Memory

// 作业之间的休眠间隔。这个时间应该在满足业务要求的前提下越长越好
const uint16_t SLEEP_SECONDS = 20;

// 正式产品中应该注释掉本句，利用条件编译的机制去掉调试代码
#define DEBUG

// 3个告警灯和蜂鸣器的引脚
#define ALARM_LED1_PIN 19
#define ALARM_LED2_PIN 18
#define ALARM_LED3_PIN 17
#define ALARM_BUZZ_PIN 25

// 使能SHT3X的IO引脚
#define TEMP_RST_TRIG_PIN 16
// SHT3X温度告警的引脚
#define TEMP_ALARM_PIN 34

// 电源管理的引脚
// 外围设备供电使能引脚
#define VDD33_EN_PIN 26
// 电源芯片模式切换引脚（高电平为PWM模式，用于唤醒时；低电平用于休眠时)
#define POWER_MODE_PIN 27
// 电源状态输入引脚（应该配置为INPUT，不要配置为OUTPUT。高电平表示电压正常，低电平表示不正常）
#define POWER_GOOD_PIN 37

// 配置WIFI
#define WIFI_SSID "TEST-CHINGO"
#define WIFI_PASSWD "9D9B148F29DC27ED11C49B9AE1"

// 使用固定IP地址，稍微提高WIFI连接速度，省电
IPAddress localIP(192, 168, 1, 113);		// 本机静态IP地址
IPAddress gateway(192, 168, 1, 1);			// 网关
IPAddress subnet(255, 255, 255, 0);			// 子网掩码
IPAddress primaryDNS(192, 168, 1, 1);		// 主DNS
IPAddress secondaryDNS(114, 114, 114, 114); // 备DNS

// mqtt配置
const char *mqtt_server = "192.168.96.37";
const int mqtt_port = 1883;
const char *mqtt_default_topic = "/topic/qos0/chingo";

/*这里后续还要做版本控制*/
// 当前版本
String localVersion = "2.27";

/*这里后续还要做设备编号设置*/
// 当前设备的设备编号
String configEquipNo = "103";

WiFiClient wifiClient;

PubSubClient mqttClient(wifiClient);

SHT3x sht(SHT3X_ADDRESS, SHT3x::Zero, TEMP_RST_TRIG_PIN);
Adafruit_SGP30 sgp;
MS5611 baro;

/*
 * 日志统一方法
 */
void PrintlnLog(String log)
{
#ifdef DEBUG
	Serial.println(log);
#endif
}

/*
 * 将传感器数据写入EEPROM
 */
void writeSensorData(SensorData sensorData)
{
	uint32_t numOfRecords = EEPROM.readUInt(EEPROM_ADDRESS_DATASIZE);
	//
	EEPROM.writeBytes(EEPROM_LEN_SENSORDATA * numOfRecords + EEPROM_LEN_DATASIZE, &sensorData, EEPROM_LEN_SENSORDATA);
	EEPROM.writeUInt(EEPROM_ADDRESS_DATASIZE, numOfRecords + 1);
	EEPROM.commit();
	numOfRecords = EEPROM.readUInt(EEPROM_ADDRESS_DATASIZE);
}

/*
 * 关闭传感器供电
 */
void powerOffSensors()
{
	// 给外围电路供电
	digitalWrite(VDD33_EN_PIN, LOW);
	// 电源进入休眠模式
	digitalWrite(POWER_MODE_PIN, LOW);
}

/* return absolute humidity [mg/m^3] with approximation formula
* @param temperature [°C]
* @param humidity [%RH]
*/
uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
	// approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
	const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
	const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);																  // [mg/m^3]
	return absoluteHumidityScaled;
}

/**
 * 发出声光报警
 * 
 * 操控LED灯和蜂鸣器以相同的节奏闪烁和鸣叫，几声短促的为一组，组之间有较长时间间隔。每一组中的循环次数表示错误代码，共可支持从1~7共7种错误代码的提示
 * CODE1（100）：温度报警;
 * CODE2（010）：TVOC报警；
 * CODE3（110）：湿度报警;
 * CODE4（001）：气压报警；
 * CODE5（101）：；
 * CODE6（011）：；
 * CODE7（111）：
 * 
 * 如果当前是报警状态（根据alarm_state），就发出报警
 */
void alarm(bool alarm1, bool alarm2, bool alarm3)
{
	if (alarm1 == true && alarm2 == true && alarm3 == true)
		return;

	// 二进制计算每组嘀的次数
	int8_t beep_count = (alarm1 ? 1 : 0) + (alarm2 ? 2 : 0) + (alarm3 ? 4 : 0);

	EasyBuzzer.setPin(ALARM_BUZZ_PIN);
	EasyBuzzer.setOnDuration(500);
	EasyBuzzer.setOffDuration(200);
	// 打开告警灯
	for (uint8_t k = 0; k < 3; k++)
	{
		digitalWrite(ALARM_LED1_PIN, alarm1 ? HIGH : LOW);
		digitalWrite(ALARM_LED2_PIN, alarm2 ? HIGH : LOW);
		digitalWrite(ALARM_LED3_PIN, alarm3 ? HIGH : LOW);
		for (int8_t i = 0; i < beep_count; i++)
		{
			EasyBuzzer.singleBeep(1000, 200);
			for (int8_t j = 0; j < 10; j++)
			{
				EasyBuzzer.update();
				delay(20);
			}
			EasyBuzzer.stopBeep();
			delay(100);
		}
		digitalWrite(ALARM_LED1_PIN, LOW);
		digitalWrite(ALARM_LED2_PIN, LOW);
		digitalWrite(ALARM_LED3_PIN, LOW);
		delay(1000);
	}
}

/**
 * 测试某个I2C地址是否有从设备响应，以便监测某个地址的设备是否正常
 */
bool testI2CAddress(uint8_t address)
{
	Wire.beginTransmission(address);
	if (Wire.endTransmission() == 0)
	{
		return true;
	}
	return false;
}
/**
 *  进行睡眠前的准备工作
 * (1) 将相关pin置为INPUT，不再输出高电平（应该不需要该工作，因为会断电）
 * (2) 将外围设备断电
 */
void prepareToSleep()
{
	WiFi.disconnect(true);
	// delay(1);
	WiFi.mode(WIFI_OFF);

	powerOffSensors();

	// 关闭MCU的无线功能（官方要求步骤）
	esp_bluedroid_disable();
	esp_bt_controller_disable();
	// WiFi.mode(WIFI_OFF);已经包含了esp_wifi_stop()
	// esp_wifi_stop();

	// 在深睡前配置好睡眠时进一步关闭RTC中的一些电源域，以达到“休眠”状态
	// 休眠状态，MCU电流<2uA
	// RTC慢速内存
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
	// RTC快速内存
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
	// RTC的协处理器、传感器和IO引脚
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
}

/**
 * 在第一次上电的时候进行声光提示，并扫描所有可用I2C设备
 * 
 * 正常情况下，应该先发出短促的嘀声，并且3盏LED灯全亮一下，然后再每盏灯逐次亮起。如果第一次全亮的时候有灯没亮，则表示
 * 这个灯坏了；如果第二次没有亮起，则表示对应的传感器设备没有响应。
 * 
 * 顺序为：(1) SHT 0X44, (2) SGP30 0X58, (3) MS5611 0X77
 */
void selfTest()
{
	// 喇叭自检
	EasyBuzzer.setPin(ALARM_BUZZ_PIN);
	EasyBuzzer.setOnDuration(500);
	EasyBuzzer.setOffDuration(200);
	EasyBuzzer.singleBeep(1200, 500);
	for (int i = 0; i < 10; i++)
	{
		EasyBuzzer.update();
		delay(50);
	}
	EasyBuzzer.stopBeep();

	// 报警灯自检，短暂闪烁0.1s
	digitalWrite(ALARM_LED1_PIN, HIGH);
	digitalWrite(ALARM_LED2_PIN, HIGH);
	digitalWrite(ALARM_LED3_PIN, HIGH);
	delay(100);
	digitalWrite(ALARM_LED1_PIN, LOW);
	digitalWrite(ALARM_LED2_PIN, LOW);
	digitalWrite(ALARM_LED3_PIN, LOW);
	delay(200);

	// I2C设备响应检查，每个响应了则对应点亮灯
	Wire.begin();
	// SHT3X
	digitalWrite(ALARM_LED1_PIN, testI2CAddress(SHT3X_ADDRESS) ? HIGH : LOW);
	delay(200);
	digitalWrite(ALARM_LED2_PIN, testI2CAddress(SGP30_ADDRESS) ? HIGH : LOW);
	delay(200);
	digitalWrite(ALARM_LED3_PIN, testI2CAddress(MS5611_ADDRESS) ? HIGH : LOW);
	delay(200);

	// 保持所有的监测状态一定时间再关掉灯，以便看清楚
	delay(500);
	digitalWrite(ALARM_LED1_PIN, LOW);
	digitalWrite(ALARM_LED2_PIN, LOW);
	digitalWrite(ALARM_LED3_PIN, LOW);

	// 告警模拟，正式产品中不要
	//alarm(true, true, true);
	//alarm(true, false, false);
}
/*
 * 初始化WIFI
 * return 耗时/ms
 */
long setupWifi()
{
	long startTime = millis();
	// We start by connecting to a WiFi network

	PrintlnLog("Connecting to ");
	PrintlnLog(WIFI_SSID);

	// 配置WIFI使用静态地址
	if (!WiFi.config(localIP, gateway, subnet, primaryDNS, secondaryDNS))
	{
		// 如果配置失败，它后面会继续使用DHCP的
		PrintlnLog("STA Failed to configure");
	}
	WiFi.persistent(false); // 不知为何，内存读写WAP会导致WIFI连接速度变慢，所以把它关掉
	WiFi.mode(WIFI_STA);	// 配置为WIFI Client模式
	WiFi.begin(WIFI_SSID, WIFI_PASSWD);

	// TODO: 后续考虑WIFI连接不上的情况处理
	while (WiFi.status() != WL_CONNECTED)
	{
		delay(5);
	}

	PrintlnLog("WiFi connected");
	PrintlnLog("IP address: ");
	PrintlnLog(WiFi.localIP().toString());
	PrintlnLog("dnsIP address: ");
	PrintlnLog(WiFi.dnsIP().toString());
	unsigned long duration = millis() - startTime;
	PrintlnLog("Connect WIFI need " + String(duration) + "ms");
	return duration;
}

/*
 * mqtt连接
 */
void mqttReconnect()
{
	// Loop until we're reconnected
	while (!mqttClient.connected())
	{
		PrintlnLog("Attempting MQTT connection...");
		// Create a random client ID
		String clientId = "ESP32Client-";
		clientId += String(random(0xffff), HEX);
		// Attempt to connect
		if (mqttClient.connect(clientId.c_str()))
		{
			PrintlnLog("mqtt connected,clientId:" + clientId);
		}
		else
		{
			PrintlnLog("failed, rc=" + String(mqttClient.state()) + " try again in 5 ms");
			delay(5);
		}
	}
}

/*
 * 初始化mqtt
 *
 * return 耗时/ms
 */
long setupMqtt()
{
	long startTime = millis();
	mqttClient.setServer(mqtt_server, mqtt_port);
	if (!mqttClient.connected())
	{
		mqttReconnect();
	}
	mqttClient.loop();
	unsigned long duration = millis() - startTime;
	PrintlnLog("Connect mqttServer need " + String(duration) + "ms");
	return duration;
}

String parsePublishData(SensorData sensorData)
{
	// StaticJsonDocument<500> json;
	// json["equipid"] = configEquipNo;
	// json["data0"] = sensorData.humidity;
	// json["data1"] = sensorData.temperature;
	// json["data2"] = sensorData.TVOC;
	// json["data3"] = sensorData.eCO2;
	// json["data4"] = sensorData.eCO2_base;
	// json["data5"] = sensorData.TVOC_base;
	// json["data6"] = sensorData.rawH2;
	// json["data7"] = sensorData.rawEthanol;
	// json["data8"] = sensorData.pressure;
	// json["data9"] = sensorData.ms5611Temperature;
	// json["data10"] = sensorData.durationWiFi;
	// json["data11"] = sensorData.durationMqtt;
	// String output = "";
	// serializeJson(json, output);
	// return output;
	String output = "";
	output += "{";
	output += "\"equipid\":";
	output += "\"" + configEquipNo + "\"";
	output += ",\"data0\":";
	output += sensorData.humidity;
	output += ",\"data1\":";
	output += sensorData.temperature;
	output += ",\"data2\":";
	output += sensorData.TVOC;
	output += ",\"data3\":";
	output += sensorData.eCO2;
	output += ",\"data4\":";
	output += sensorData.eCO2_base;
	output += ",\"data5\":";
	output += sensorData.TVOC_base;
	output += ",\"data6\":";
	output += sensorData.rawH2;
	output += ",\"data7\":";
	output += sensorData.rawEthanol;
	output += ",\"data8\":";
	output += sensorData.pressure;
	output += ",\"data9\":";
	output += sensorData.ms5611Temperature;
	output += ",\"data10\":";
	output += sensorData.durationWiFi;
	output += ",\"data11\":";
	output += sensorData.durationMqtt;
	output += ",\"data12\":";
	output += sensorData.index;
	output += "}";
	return output;
}

/*
 * mqtt发送数据
 */
long publishData(SensorData sensorData)
{
	long startTime = millis();
	String data = parsePublishData(sensorData);
	const char *payload = data.c_str();
	PrintlnLog(payload);
	if (mqttClient.publish(mqtt_default_topic, payload))
	{
		PrintlnLog("publish success.");
	}
	else
	{
		PrintlnLog("publish failed!");
	}
	unsigned long duration = millis() - startTime;
	PrintlnLog("publish onedata to mqttServer need " + String(duration) + "ms");
	return duration;
}

/**
 * 进行一次采集数据的业务工作
 */
SensorData getDataOnce()
{
	// 初始化传感器
	sht.Begin();
	baro = MS5611();
	baro.begin();
	if (!sgp.begin())
	{
		PrintlnLog("SGP30 not found");
	}
	else
	{
		PrintlnLog("Found SGP30 serial >>>0x" + String(sgp.serialnumber[0], HEX) + " 0x" + String(sgp.serialnumber[1], HEX) + " 0x" + String(sgp.serialnumber[2], HEX));
	}

	// 各个需要采集的数据
	SensorData sensorData;

	sht.UpdateData();
	uint8_t r = sht.GetError();
	if (r == 0)
	{
		sensorData.temperature = sht.GetTemperature();
		sensorData.humidity = sht.GetRelHumidity();
	}

	if (sgp.IAQmeasure())
	{
		sensorData.TVOC = sgp.TVOC;
		sensorData.eCO2 = sgp.eCO2;
	}

	if (sgp.IAQmeasureRaw())
	{
		sensorData.rawH2 = sgp.rawH2;
		sensorData.rawEthanol = sgp.rawEthanol;
	}

	if (!sgp.getIAQBaseline(&sensorData.eCO2_base, &sensorData.TVOC_base))
	{
		PrintlnLog("Failed to get baseline readings");
	}

	sensorData.pressure = baro.getPressure();
	sensorData.ms5611Temperature = (float)(baro.getTemperature()) / 100;

	// 关掉传感器省电
	digitalWrite(VDD33_EN_PIN, LOW);

	// TODO: 告警规则后续优化
	// 阻塞式告警代码
	bool alarm1 = false, alarm2 = false, alarm3 = false;
	if (sensorData.temperature >= 32)
	{
		alarm1 = true;
	}
	else
	{
		alarm1 = false;
	}
	if (alarm1 != false || alarm2 != false || alarm3 != false)
	{
		alarm(alarm1, alarm2, alarm3);
	}

	// SHT3X
	PrintlnLog("SHT3X:");
	PrintlnLog("Temperature: " + String(sensorData.temperature) + "\xC2\xB0" + "C" + String("\tHumidity: ") + String(sensorData.humidity) + "%");

	// SGP30
	PrintlnLog("SGP30:");
	PrintlnLog("TVOC: " + String(sensorData.TVOC) + " ppb\t eCO2: " + String(sensorData.eCO2) + " ppm\tRaw H2: " + String(sensorData.rawH2) + " \tRaw Ethanol: " + String(sensorData.rawEthanol));
	PrintlnLog("****Baseline values: eCO2: " + String(sensorData.eCO2_base) + " & TVOC: " + String(sensorData.TVOC_base));

	// MS5611
	PrintlnLog("MS5611:");
	PrintlnLog("Pressure: " + String(sensorData.pressure) + "\tTemperature: " + String(sensorData.ms5611Temperature));

	return sensorData;
}

// 发送请求，提交数据
void postData()
{
	uint32_t numOfRecords = EEPROM.readUInt(EEPROM_ADDRESS_DATASIZE);
	PrintlnLog("numOfRecords:" + String(numOfRecords));
	if (numOfRecords != DATASIZE_POST)
	{
		return;
	}

	long durationWiFi = setupWifi();
	long durationMqtt = setupMqtt();
	for (size_t i = 0; i < numOfRecords; i++)
	{
		SensorData sensorData;
		EEPROM.readBytes(EEPROM_LEN_SENSORDATA * i + EEPROM_LEN_DATASIZE, &sensorData, EEPROM_LEN_SENSORDATA);
		sensorData.index = i + 1;
		if (i == 0)
		{
			sensorData.durationWiFi = durationWiFi;
			sensorData.durationMqtt = durationMqtt;
		}
		publishData(sensorData);
	}
	// 发送一个空的消息过去，通过这种方式暂时解决最后一组传感器数据没有传送过去的问题
	mqttClient.publish(mqtt_default_topic, "");

	EEPROM.writeUInt(EEPROM_ADDRESS_DATASIZE, 0);
	EEPROM.commit();
}
/*
 * 进行一次工作
 */
void doJob()
{
	SensorData sensorData = getDataOnce();
	writeSensorData(sensorData);
	postData();
}

void setup()
{
	// delay(5000);
	unsigned long startTime;
	startTime = millis();

	// 只有需要上报数据的时候再根据需要酌情打开蓝牙或WIFI。用不到的就一直保持关闭状态
	esp_bluedroid_disable();
	esp_bt_controller_disable();
	esp_wifi_stop();

	// 将电源改为PWM模式
	pinMode(POWER_MODE_PIN, OUTPUT);
	digitalWrite(POWER_MODE_PIN, HIGH);
	// 给外围电路供电
	pinMode(VDD33_EN_PIN, OUTPUT);
	digitalWrite(VDD33_EN_PIN, HIGH);
	// 等待一定的时间，让外围电路就位
	// delay(500);
	delay(5);

	// 初始化引脚
	pinMode(ALARM_LED1_PIN, OUTPUT);
	pinMode(ALARM_LED2_PIN, OUTPUT);
	pinMode(ALARM_LED3_PIN, OUTPUT);

	// 使能SHT3X
	pinMode(TEMP_RST_TRIG_PIN, OUTPUT);
	digitalWrite(TEMP_RST_TRIG_PIN, LOW);

#ifdef DEBUG
	Serial.begin(9600);
	while (!Serial)
	{
	}
#endif
	if (EEPROM.begin(EEPROM_LEN_SENSORDATA * DATASIZE_POST + EEPROM_LEN_DATASIZE))
	{
		PrintlnLog("EEPROM.begin success.");
	}
	else
	{
		PrintlnLog("EEPROM.begin fail");
	}

	// 如果是第一次上电则开机自检
	if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED)
	{
		// 第一次上电要将dataSize置为0
		EEPROM.writeUInt(EEPROM_ADDRESS_DATASIZE, 0);
		EEPROM.commit();
		// 自检
		selfTest();
	}

	// 设定下一次换醒（单位微秒，1S = 10^6μS
	esp_sleep_enable_timer_wakeup(SLEEP_SECONDS * 1000000);

	doJob();

	// 计算执行时间和占空比，用于评估电池寿命（假定工作电流和休眠电流已知，通过累计使用电量，即可知道电池剩余电量）
	unsigned long duration = millis() - startTime;
	float dutyRate = (float)duration / (float)(SLEEP_SECONDS * 1000 / 100);

	PrintlnLog("DUTY RATIO:");
	PrintlnLog("Executed in " + String(duration) + " millseconds, and duty ratio is: " + String(dutyRate) + "%");
	PrintlnLog("----------------------------------------------------------------");

	// 进入睡眠，需要改为休眠模式（hibernation）
	prepareToSleep();
	esp_deep_sleep_start();
}

// 这个函数不会被执行到。每次执行到setup函数结尾的时候就会进入睡眠
void loop()
{
	PrintlnLog("This loop log will never print...");
}