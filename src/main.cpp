#include <Arduino.h>
#include <Wire.h>

#include "EasyBuzzer.h"
#include "Adafruit_SGP30.h"
#include "MS5611.h"

#include "SHT3x.h"

#define SHT3X_ADDRESS 0x44

#define DEBUG

#ifdef DEBUG
#define CG_DEBUG_PRINT(x) Serial.println(x)
#else
#define CG_DEBUG_PRINT(x)
#endif

// pin指的都是GPIO的PIN编号，而不是管脚序号
#define ALARM_LED_PIN 18
#define ALARM_BUZZ_PIN 25
#define TEMP_RST_TRIG_PIN 16

SHT3x sht(SHT3X_ADDRESS, SHT3x::Zero, TEMP_RST_TRIG_PIN);
Adafruit_SGP30 sgp;
MS5611 baro;

int32_t pressure;

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
 * 以一定的频率发出声光报警
 * 
 * 如果当前是报警状态（根据alarm_state），就发出报警
 */
void alarm(bool alert1, bool alert2, bool alert3) {
	if (alert1 == true && alert2 == true && alert3 == true) return;

	// 二进制计算每组嘀的次数
	int8_t alert_count = (alert1 ? 1 : 0) + (alert2 ? 2 : 0) + (alert3 ? 4 : 0);

	EasyBuzzer.setPin(ALARM_BUZZ_PIN);
	EasyBuzzer.setOnDuration(500);
	EasyBuzzer.setOffDuration(200);
	// 打开告警灯
	for (uint8_t k = 0; k < 3; k ++) {
		digitalWrite(19, alert1 ? HIGH : LOW);
		digitalWrite(18, alert2 ? HIGH : LOW);
		digitalWrite(17, alert3 ? HIGH : LOW);
		for (int8_t i = 0; i < alert_count; i ++) {
			EasyBuzzer.singleBeep(1000, 200);
			for (int8_t j = 0; j < 10; j ++) {
				EasyBuzzer.update();
				delay(20);
			}
			EasyBuzzer.stopBeep();
			delay(100);
		}
		digitalWrite(19, LOW);
		digitalWrite(18, LOW);
		digitalWrite(17, LOW);
		delay(1000);
	}
}

bool testI2CAddress(uint8_t address) {
	Wire.beginTransmission(address);
	if (Wire.endTransmission() == 0) {
		return true;
	}
	return false;
}
/**
 *  进行睡眠前的准备工作
 * (1) 将相关pin置为INPUT，不再输出高电平（应该不需要该工作，因为会断电）
 * (2) 将外围设备断电
 */
void sleep_prepare()
{
	digitalWrite(27, LOW);
	digitalWrite(26, LOW);
}

/**
 * 在第一次上电的时候进行声光提示，并扫描所有可用I2C设备
 * 
 * 正常情况下，应该先发出短促的嘀声，并且3盏LED灯全亮一下，然后再每盏灯逐次亮起。如果第一次全亮的时候有灯没亮，则表示
 * 这个灯坏了；如果第二次没有亮起，则表示对应的传感器设备没有响应。
 * (1) SHT 0X44, (2) SGP30 0X58, (3) MS5611 0X77
 */
void powerOnIndicate() {
	// 喇叭自检
	EasyBuzzer.setPin(ALARM_BUZZ_PIN);
	EasyBuzzer.setOnDuration(500);
	EasyBuzzer.setOffDuration(200);
	EasyBuzzer.singleBeep(1000, 500);
	for (int i = 0; i < 10; i ++) {
		EasyBuzzer.update();
		delay(50);
	}
	EasyBuzzer.stopBeep();

	// 报警灯自检
	digitalWrite(17, HIGH);
	digitalWrite(18, HIGH);
	digitalWrite(19, HIGH);
	delay(100);
	digitalWrite(17, LOW);
	digitalWrite(18, LOW);
	digitalWrite(19, LOW);
	delay(200);

	// I2C设备响应检查，每个响应了则对应点亮灯
	Wire.begin();
	// SHT3X
	if (testI2CAddress(0x44)) {
		digitalWrite(19, HIGH);
	}
	else {
		digitalWrite(19, LOW);
	}
	delay(200);
	// SGP30
	if (testI2CAddress(0x58)) {
		digitalWrite(18, HIGH);
	}
	else {
		digitalWrite(18, LOW);
	}
	delay(200);
	
	// MS5611
	if (testI2CAddress(0x77)) {
		digitalWrite(17, HIGH);
	}
	else {
		digitalWrite(17, LOW);
	}
	delay(200);
	delay(500);
	digitalWrite(19, LOW);
	digitalWrite(18, LOW);
	digitalWrite(17, LOW);

	// 告警模拟，正式产品中不要
	//alarm(true, true, true);
	//alarm(true, false, false);
}

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : 
		Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason);
		break;
  }
  if (wakeup_reason == 0) {
	powerOnIndicate();
  }
}


int counter = 0;
void getDataOnce()
{
	// 各个需要采集的数据
	float temperature;

	sht.UpdateData();
	uint8_t r = sht.GetError();
	if (r != 0)
	{
		printf("Error %d\n", r);
		sht.HardReset();
		for (int i = 0; i < r; i++)
		{
			digitalWrite(ALARM_LED_PIN, HIGH);
			delay(50);
			digitalWrite(ALARM_LED_PIN, LOW);
			delay(50);
		}
	}
	else
	{
		Serial.print("Temperature: ");
		temperature = sht.GetTemperature();
		Serial.print(temperature);
		Serial.write("\xC2\xB0"); //The Degree symbol
		Serial.println("C");
		Serial.print("Humidity: ");
		Serial.print(sht.GetRelHumidity());
		Serial.println("%");
	}

	if (!sgp.IAQmeasure())
	{
		Serial.println("Measurement failed");
		return;
	}
	Serial.print("TVOC ");
	Serial.print(sgp.TVOC);
	Serial.print(" ppb\t");
	Serial.print("eCO2 ");
	Serial.print(sgp.eCO2);
	Serial.println(" ppm");

	if (!sgp.IAQmeasureRaw())
	{
		Serial.println("Raw Measurement failed");
		return;
	}
	Serial.print("Raw H2 ");
	Serial.print(sgp.rawH2);
	Serial.print(" \t");
	Serial.print("Raw Ethanol ");
	Serial.print(sgp.rawEthanol);
	Serial.println("");

	uint16_t TVOC_base, eCO2_base;
	if (!sgp.getIAQBaseline(&eCO2_base, &TVOC_base))
	{
		Serial.println("Failed to get baseline readings");
		return;
	}
	Serial.print("****Baseline values: eCO2: 0x");
	Serial.print(eCO2_base, HEX);
	Serial.print(" & TVOC: 0x");
	Serial.println(TVOC_base, HEX);
	
	pressure = baro.getPressure();
	// Send pressure via serial (UART);
	Serial.print("Pressure: ");
	Serial.println(pressure);
	Serial.print("Temperature: ");
	Serial.println((float)(baro.getTemperature()) / 100);
	Serial.println("-------------------------------------");

	// 阻塞式告警代码
	bool alarm1 = false, alarm2 = false, alarm3 = false;
	if (temperature >= 32) {
		alarm1 = true;
	}
	else {
		alarm1 = false;
	}
	if (alarm1 != false || alarm2 != false || alarm3 != false) {
		alarm(alarm1, alarm2, alarm3);
	}
}

const uint16_t SLEEP_SECONDS = 30;
unsigned long startTime;
void setup()
{
	startTime = millis();

	// 将电源改为PWM模式
	pinMode(27, OUTPUT);
	digitalWrite(27, HIGH);
	// 给外围电路供电
	pinMode(26, OUTPUT);
	digitalWrite(26, HIGH);

	pinMode(ALARM_LED_PIN, OUTPUT);
	pinMode(17, OUTPUT);
	pinMode(19, OUTPUT);

	// 使能SHT3X
	pinMode(TEMP_RST_TRIG_PIN, OUTPUT);
	digitalWrite(TEMP_RST_TRIG_PIN, LOW);

	Serial.begin(9600);
	while (!Serial) {}

	// 等待一定的时间，让外围电路就位
	delay(5);
	print_wakeup_reason();
	
	// 设定下一次换醒（每30秒唤醒一次）
  	esp_sleep_enable_timer_wakeup(SLEEP_SECONDS * 1000000);

	sht.Begin();
	
	baro = MS5611();
	baro.begin();


	if (!sgp.begin())
	{
		Serial.println("sht not found :(");
	}
	else {
		Serial.print("Found SGP30 serial #");
		Serial.print(sgp.serialnumber[0], HEX);
		Serial.print(sgp.serialnumber[1], HEX);
		Serial.println(sgp.serialnumber[2], HEX);
	}
	getDataOnce();
  	Serial.flush();
	sleep_prepare();
	unsigned long duration = millis() - startTime;
	Serial.print("Executed in ");
	Serial.print(duration);
	Serial.println(" millseconds.");
	Serial.print("Duty ratio is: ");
	Serial.print((float)duration / (float)(SLEEP_SECONDS * 1000 / 100));
	Serial.println("%");
  	esp_deep_sleep_start();
}

// 这个函数不会被执行到。每次执行到setup函数结尾的时候就会进入睡眠
void loop() {}