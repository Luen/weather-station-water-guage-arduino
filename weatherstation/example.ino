#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h> // Use <WiFi.h> for ESP32
#include <PubSubClient.h>

// WiFi credentials
const char *ssid = "yourSSID";
const char *password = "yourPassword";

// MQTT Broker settings
const char *mqtt_broker = "mqtt_broker_address";
const char *mqtt_username = ""; // Leave blank if not applicable
const char *mqtt_password = ""; // Leave blank if not applicable
const int mqtt_port = 1883;
const char *mqtt_topic = "sensor/bme280";

// Initialize BME280 sensor
Adafruit_BME280 bme; // I2C

// Initialize WiFi and MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

void setup()
{
    Serial.begin(115200);

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Connect to MQTT Broker
    client.setServer(mqtt_broker, mqtt_port);
    while (!client.connected())
    {
        Serial.println("Connecting to MQTT...");
        if (client.connect("ESP8266Client", mqtt_username, mqtt_password))
        {
            Serial.println("Connected to MQTT");
        }
        else
        {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }

    // Initialize BME280 sensor
    if (!bme.begin())
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1)
            ;
    }
}

void loop()
{
    if (!client.connected())
    {
        while (!client.connected())
        {
            Serial.println("Reconnecting to MQTT...");
            if (client.connect("ESP8266Client", mqtt_username, mqtt_password))
            {
                Serial.println("Connected to MQTT");
            }
            else
            {
                Serial.print("failed with state ");
                Serial.print(client.state());
                delay(2000);
            }
        }
    }
    client.loop();

    // Read sensor data
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0F; // Convert to hPa

    // Create data string
    char data[100];
    snprintf(data, sizeof(data), "Temperature: %f°C, Humidity: %f%%, Pressure: %f hPa", temperature, humidity, pressure);
    Serial.println(data);

    // Publish data to MQTT topic
    client.publish(mqtt_topic, data);

    delay(5000); // Wait 5 seconds before next read
}
