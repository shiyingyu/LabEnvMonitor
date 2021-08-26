#include <Arduino.h>
#include <Wire.h>

#include "EasyBuzzer.h"
#include "Adafruit_SGP30.h"

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
#define TEMP_RST_TRIG_PIN 17

SHT3x sensor(SHT3X_ADDRESS, SHT3x::Zero, TEMP_RST_TRIG_PIN);
Adafruit_SGP30 sgp;

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
bool alarm_state = false;
bool already_in_beep = false;
void alarm() {
	if (!alarm_state) {
		digitalWrite(ALARM_LED_PIN, LOW);
		EasyBuzzer.stopBeep();
	}
	else {
		if (!already_in_beep) {
			already_in_beep = true;
			EasyBuzzer.beep(1500, 200, 500, 127, 1000, 0);
		}
	}
	
	EasyBuzzer.update();
}

void Scanner()
{
	Serial.println();
	Serial.println("I2C scanner. Scanning ...");
	byte count = 0;

	Wire.begin();
	for (uint8_t i = 8; i <= 127; i++)
	{
		Wire.beginTransmission(i);		 // Begin I2C transmission Address (i)
		if (Wire.endTransmission() == 0) // Receive 0 = success (ACK response)
		{
			Serial.print("Found address: ");
			Serial.print(i, DEC);
			Serial.print(" (0x");
			Serial.print(i, HEX); // PCF8574 7 bit address
			Serial.println(")");
			count++;
		}
	}
	Serial.print("Found ");
	Serial.print(count, DEC); // numbers of devices
	Serial.println(" device(s).");
}

void setup()
{
	Serial.begin(9600);
	delay(500);
	sensor.Begin();
	delay(200);
	pinMode(ALARM_LED_PIN, OUTPUT);
	// 报警灯自检
	digitalWrite(ALARM_LED_PIN, HIGH);
	delay(500);
	digitalWrite(ALARM_LED_PIN, LOW);
	// 喇叭自检
	EasyBuzzer.setPin(ALARM_BUZZ_PIN);
	EasyBuzzer.setOnDuration(500);
	EasyBuzzer.setOffDuration(200);
	EasyBuzzer.singleBeep(1000, 200);
	delay(200);
	EasyBuzzer.stopBeep();

	// I2C设备扫描，输出到串口
	Scanner();

	// 使能SHT3X
	pinMode(TEMP_RST_TRIG_PIN, OUTPUT);
	digitalWrite(TEMP_RST_TRIG_PIN, LOW);

	if (!sgp.begin())
	{
		Serial.println("Sensor not found :(");
		while (1)
			;
	}
	Serial.print("Found SGP30 serial #");
	Serial.print(sgp.serialnumber[0], HEX);
	Serial.print(sgp.serialnumber[1], HEX);
	Serial.println(sgp.serialnumber[2], HEX);
}

int counter = 0;
void loop()
{
	alarm();
	sensor.UpdateData();
	uint8_t r = sensor.GetError();
	if (r != 0)
	{
		printf("Error %d\n", r);
		sensor.HardReset();
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
		float temperature = sensor.GetTemperature();
		Serial.print(temperature);
		Serial.write("\xC2\xB0"); //The Degree symbol
		Serial.println("C");
		Serial.print("Humidity: ");
		Serial.print(sensor.GetRelHumidity());
		Serial.println("%");
		if (temperature >= 34) {
			alarm_state = true;
		}
		else {
			alarm_state = false;
		}
	}
	delay(1000);
	delay(500);

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

	delay(1000);

	counter++;
	if (counter == 30)
	{
		counter = 0;

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
	}
}