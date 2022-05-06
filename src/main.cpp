#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <secrets.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

unsigned long delayTime = 5000;
unsigned long lastPublish;

WiFiClientSecure wiFiClient;
void msgReceived(char *topic, byte *payload, unsigned int len);
PubSubClient pubSubClient(awsEndpoint, 8883, msgReceived, wiFiClient);

Adafruit_BME280 bme; // I2C

void checkBME280()
{
  bool status;
  status = bme.begin(0x76);
  if (!status)
  {
    Serial.println("Could not find valid BME280 sensor, check wiring!");
    while (1)
      ;
  }
  Serial.println();
}

float getTemp()
{
  return bme.readTemperature();
}

float getHumidity()
{
  return bme.readHumidity();
}

void printSensorValues(float temp, float humid)
{
  Serial.printf("Temperature = %f *C\n", temp);
  Serial.printf("Humidity = %f %\n", humid);
}

void pubSubCheckConnect()
{
  if (!pubSubClient.connected())
  {
    Serial.print("PubSubClient connecting to: ");
    Serial.print(awsEndpoint);
    while (!pubSubClient.connected())
    {
      Serial.print(".");
      pubSubClient.connect("ESPBME280");
      delay(1000);
    }
    Serial.println(" connected");
    pubSubClient.subscribe("espbme280in");
  }
  pubSubClient.loop();
}

void msgReceived(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message received on ");
  Serial.print(topic);
  Serial.print(": ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void connectWiFi()
{
  Serial.print("Connecting to ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);
  WiFi.waitForConnectResult();
  Serial.println(", WiFi connected, IP address: ");
  Serial.println(WiFi.localIP());

  wiFiClient.setCACert(ca);
  wiFiClient.setCertificate(pemCert);
  wiFiClient.setPrivateKey(key);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  };

  Serial.println();
  Serial.println("ESP32 AWS IoT Test");
  Serial.printf("SDK version: %s\n", ESP.getSdkVersion());

  checkBME280();
  connectWiFi();
}

void loop()
{
  pubSubCheckConnect();

  // if you need to increase buffer size, you need to change MQTT_MAX_PACKET_SIZE in PubSubClient.h
  char payload[128];

  if (millis() - lastPublish > delayTime)
  {
    float t = getTemp();
    float h = getHumidity();
    printSensorValues(t, h);
    sprintf(payload, "{\"uptime\":%lu,\"temp\":%f,\"humid\":%f}", millis(), t, h);
    bool rc = pubSubClient.publish("espbme280out", payload);
    Serial.print("Published, rc=");
    Serial.print((rc ? "OK: " : "FAILED: "));
    Serial.println(payload);
    lastPublish = millis();
  }
}