#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME280.h"

Adafruit_BME280 bme;

const char *ssid = "XXXX";
const char *password = "XXXX";
const char *mqtt_server = "XXX.XXX.X.XX";
String clientId = "WeatherStation-";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi()
{

  delay(100);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.init(AP_STA_MODE);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *payload, unsigned int length)
{
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println(F("BME280 test"));

  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME280_OS_8X);
  bme.setHumidityOversampling(BME280_OS_2X);
  bme.setPressureOversampling(BME280_OS_4X);
  bme.setIIRFilterSize(BME280_FILTER_SIZE_3);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop()
{

  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  StaticJsonDocument<32> doc;
  char output[55];

  long now = millis();
  if (now - lastMsg > 5000)
  {
    lastMsg = now;
    float temp = bme.readTemperature();
    float pressure = bme.readPressure() / 100.0;
    float humidity = bme.readHumidity();
    doc["t"] = temp;
    doc["p"] = pressure;
    doc["h"] = humidity;
    Serial.println("Read");

    serializeJson(doc, output);
    Serial.println(output);
    client.publish("/home/sensors", output);
    Serial.println("Sent");
  }
}