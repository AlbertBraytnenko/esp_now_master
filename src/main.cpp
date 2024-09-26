#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "quaternion.h"

using namespace imu;

#define CHANNEL 1

esp_now_peer_info_t slave;
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

Quaternion orientation, qDeviation;
Vector<3> eulerAngles;

float lastSendTime = 0;
double Bias[3];
int data[2];

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void ScanForSlave();
int RadToDeg(double radVal);
void LEDBlinking(int ledPin, int numOfBlinks, int blinkDuration, bool counting);

void setup()
{
	Serial.begin(115200);
	WiFi.mode(WIFI_STA);
	esp_now_init();
	esp_now_register_send_cb(OnDataSent);
	ScanForSlave();
	esp_now_add_peer(&slave);
	pinMode(LED_BUILTIN, OUTPUT);

	if (!mpu.begin())
	{
		Serial.println("Failed to find MPU6050 chip");
		while (1)
		{
			delay(10);
		}
	}

	mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
	mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
	mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
	delay(100);

	Serial.println("Calibrate MPU6050 strats in...");
	LEDBlinking(LED_BUILTIN, 5, 500, 1);

	digitalWrite(LED_BUILTIN, HIGH);

	int loopInt = 850;
	for (int i = 0; i < loopInt; i++)
	{
		mpu.getEvent(&a, &g, &temp);
		Bias[0] += g.gyro.x;
		Bias[1] += g.gyro.y;
		Bias[2] += g.gyro.z;
	}
	Bias[0] /= loopInt;
	Bias[1] /= loopInt;
	Bias[2] /= loopInt;

	digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
	if ((millis() - lastSendTime) >= 10)
	{
		mpu.getEvent(&a, &g, &temp);

		Quaternion qW(0, (g.gyro.x - Bias[0]), (g.gyro.y - Bias[1]), (g.gyro.z - Bias[2]));

		qDeviation = orientation * qW * 0.5;

		orientation = orientation + qDeviation * 0.01;
		orientation.normalize();

		eulerAngles = orientation.toEuler();

		int eueueX = RadToDeg(eulerAngles.x());
		int eueueZ = RadToDeg(eulerAngles.z());

		if (eueueX >= 0 && eueueX <= 180)
			data[1] = eueueX;
		if (eueueZ >= 0 && eueueZ <= 180)
			data[0] = eueueZ;

		esp_now_send(slave.peer_addr, (uint8_t *)data, sizeof(data));
		lastSendTime = millis();
	}
}

void ScanForSlave()
{
	int8_t scanResults = WiFi.scanNetworks();

	for (int i = 0; i < scanResults; ++i)
	{
		String SSID = WiFi.SSID(i);
		String BSSIDstr = WiFi.BSSIDstr(i);

		if (SSID.indexOf("RXNO") == 0)
		{

			int mac[6];
			if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
				for (int ii = 0; ii < 6; ++ii)
					slave.peer_addr[ii] = (uint8_t)mac[ii];
					
			slave.channel = CHANNEL;
			slave.encrypt = 0;
			break;
		}
	}
}

/** callback when data is sent from Master to Slave **/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
	for (int i = 0; i < 2; i++)
	{
		Serial.print(data[i]);
		Serial.print("\t");
	}
	Serial.println("");
}

int RadToDeg(double radVal)
{
	return int((radVal * (180.0 / M_PI)));
}

void LEDBlinking(int ledPin, int numOfBlinks, int blinkDuration, bool counting)
{
	for (int i = 1; i <= numOfBlinks; i++)
	{
		if (counting)
		{
			Serial.print(i);
			Serial.print("\t");
		}
		digitalWrite(ledPin, HIGH);
		delay(blinkDuration/2);
		digitalWrite(ledPin, LOW);
		delay(blinkDuration/2);
	}
}