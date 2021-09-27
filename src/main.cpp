#include <Arduino.h>
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_wifi.h"
#include <Wire.h>

#include "EasyBuzzer.h"
#include "Adafruit_SGP30.h"
#include "MS5611.h"

#include "SHT3x.h"

#define SHT3X_ADDRESS 0x44
#define SGP30_ADDRESS 0x58
#define MS5611_ADDRESS 0X77

// 作业之间的休眠间隔。这个时间应该在满足业务要求的前提下越长越好
const uint16_t SLEEP_SECONDS = 30;

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

SHT3x sht(SHT3X_ADDRESS, SHT3x::Zero, TEMP_RST_TRIG_PIN);
Adafruit_SGP30 sgp;
MS5611 baro;

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
	digitalWrite(POWER_MODE_PIN, LOW);
	digitalWrite(VDD33_EN_PIN, LOW);

	// 关闭MCU的无线功能（官方要求步骤）
	esp_bluedroid_disable();
	esp_bt_controller_disable();
	esp_wifi_stop();

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

/**
 * 进行一次采集数据的业务工作
 */
void getDataOnce()
{
	// 各个需要采集的数据
	float temperature, humidity;
	uint16_t TVOC, eCO2, eCO2_base, TVOC_base, rawH2, rawEthanol;
	int32_t pressure;
	float ms5611Temperature;

	sht.UpdateData();
	uint8_t r = sht.GetError();
	if (r == 0)
	{
		temperature = sht.GetTemperature();
		humidity = sht.GetRelHumidity();
	}

	if (sgp.IAQmeasure())
	{
		TVOC = sgp.TVOC;
		eCO2 = sgp.eCO2;
	}

	if (sgp.IAQmeasureRaw())
	{
		rawH2 = sgp.rawH2;
		rawEthanol = sgp.rawEthanol;
	}

	if (!sgp.getIAQBaseline(&eCO2_base, &TVOC_base))
	{
		Serial.println("Failed to get baseline readings");
	}

	pressure = baro.getPressure();
	ms5611Temperature = (float)(baro.getTemperature()) / 100;

	// 阻塞式告警代码
	bool alarm1 = false, alarm2 = false, alarm3 = false;
	if (temperature >= 32)
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
#ifdef DEBUG
	// SHT3X
	Serial.println("SHT3X:");
	Serial.print("Temperature: ");
	Serial.print(temperature);
	Serial.write("\xC2\xB0"); //The Degree symbol
	Serial.print("C");
	Serial.print("\tHumidity: ");
	Serial.print(humidity);
	Serial.println("%");
	Serial.println("");

	// SGP30
	Serial.println("SGP30:");
	Serial.print("TVOC: ");
	Serial.print(TVOC);
	Serial.print(" ppb\t eCO2: ");
	Serial.print(eCO2);
	Serial.print(" ppm\tRaw H2: ");
	Serial.print(rawH2);
	Serial.print(" \tRaw Ethanol: ");
	Serial.print(rawEthanol);
	Serial.println("");
	Serial.print("****Baseline values: eCO2: 0x");
	Serial.print(eCO2_base, HEX);
	Serial.print(" & TVOC: 0x");
	Serial.println(TVOC_base, HEX);
	Serial.println("");

	// MS5611
	Serial.println("MS5611:");
	Serial.print("Pressure: ");
	Serial.print(pressure);
	Serial.print("\tTemperature: ");
	Serial.println(ms5611Temperature);
	Serial.println("");
#endif
}

void setup()
{
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
	delay(5);

	// 初始化引脚
	pinMode(ALARM_LED1_PIN, OUTPUT);
	pinMode(ALARM_LED2_PIN, OUTPUT);
	pinMode(ALARM_LED3_PIN, OUTPUT);

	// 使能SHT3X
	pinMode(TEMP_RST_TRIG_PIN, OUTPUT);
	digitalWrite(TEMP_RST_TRIG_PIN, LOW);

	Serial.begin(9600);
	while (!Serial)
	{
	}

	// 如果是第一次上电则开机自检
	if (esp_sleep_get_wakeup_cause() == 0)
	{
		selfTest();
	}

	// 设定下一次换醒（单位微秒，1S = 10^6μS
	esp_sleep_enable_timer_wakeup(SLEEP_SECONDS * 1000000);

	sht.Begin();
	baro = MS5611();
	baro.begin();

	if (!sgp.begin())
	{
		Serial.println("SGP30 not found");
	}
	else
	{
#ifdef DEBUG
		Serial.print("Found SGP30 serial #");
		Serial.print(sgp.serialnumber[0], HEX);
		Serial.print(sgp.serialnumber[1], HEX);
		Serial.println(sgp.serialnumber[2], HEX);
#endif
	}
	getDataOnce();

	// 计算执行时间和占空比，用于评估电池寿命（假定工作电流和休眠电流已知，通过累计使用电量，即可知道电池剩余电量）
	unsigned long duration = millis() - startTime;
	float dutyRate = (float)duration / (float)(SLEEP_SECONDS * 1000 / 100);
#ifdef DEBUG
	Serial.println("DUTY RATION:");
	Serial.print("Executed in ");
	Serial.print(duration);
	Serial.println(" millseconds, and duty ratio is: ");
	Serial.print(dutyRate);
	Serial.println("%");
	Serial.println("----------------------------------------------------------------");

	Serial.flush();
#endif

	// 进入睡眠，需要改为休眠模式（hibernation）
	prepareToSleep();
	esp_deep_sleep_start();
}

// 这个函数不会被执行到。每次执行到setup函数结尾的时候就会进入睡眠
void loop() {}