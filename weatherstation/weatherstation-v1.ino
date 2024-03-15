#include <MKRNB.h>
#include <ArduinoMqttClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

#define TRIG_PIN 5
#define ECHO_PIN 6

#define rxPin 10
#define txPin 11

// Cellular credentials
const char PINNUMBER[] = "";                // Pin number if SIM is PIN protected
const char GPRS_APN[] = "telstra.internet"; // Telstra APN
const char GPRS_LOGIN[] = "";               // Leave blank if not used
const char GPRS_PASSWORD[] = "";            // Leave blank if not used

// MQTT Broker settings
const char mqtt_broker[] = "mqtt_broker_address";
const int mqtt_port = 1883;
const char mqtt_topic[] = "sensor/bme280";

// Initialize the JSN-SR04T sensor
SoftwareSerial jsnSerial(rxPin, txPin);

// Initialize the BME280 sensor
Adafruit_BME280 bme; // I2C

// Initialize the cellular hardware
NB nbAccess;
GPRS gprs;
NBClient nbClient;
MqttClient mqttClient(nbClient);

void setup()
{
    jsnSerial.begin(9600);
    Serial.begin(115200);

    // Ultrasonic sensor
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW); // Ensure trigger pin is low for ultrasonic sensor
    pinMode(ECHO_PIN, INPUT);

    // Start the BME280 sensor
    if (!bme.begin())
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1)
            ;
    }

    Serial.println("Attempting to connect to the cellular network");

    // Start connection to the cellular network
    if (nbAccess.begin(PINNUMBER) != NB_READY || gprs.attachGPRS(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD) != GPRS_READY)
    {
        Serial.println("Failed to connect to the cellular network");
        while (1)
            ;
    }

    Serial.println("Connected to the cellular network");

    // Connect to the MQTT Broker
    Serial.print("Connecting to MQTT broker...");
    mqttClient.setId("MKRNB1500Client");
    mqttClient.setUsernamePassword("mqtt_username", "mqtt_password"); // Update with your MQTT credentials

    while (!mqttClient.connect(mqtt_broker, mqtt_port))
    {
        Serial.print(".");
        delay(5000); // Wait 5 seconds before retrying
    }

    Serial.println("\nConnected to the MQTT broker");
}

int getDistance()
{
    jsnSerial.write(0x01); // Wake up the sensor
    delay(50);             // Short delay to allow the sensor to respond

    if (jsnSerial.available())
    {
        unsigned int distance;
        byte startByte, h_data, l_data, sum = 0;
        byte buf[3];

        startByte = (byte)jsnSerial.read();
        if (startByte == 255)
        {
            jsnSerial.readBytes(buf, 3);
            h_data = buf[0];
            l_data = buf[1];
            sum = buf[2];
            distance = (h_data << 8) + l_data;

            if (((h_data + l_data) & 0xFF) == sum)
            {
                return distance; // Return distance if data is valid
            }
        }
    }
    return -1; // Return -1 or some other error indicator if no valid data is received
}

void loop()
{
    if (!mqttClient.connected())
    {
        Serial.print("Reconnecting to MQTT broker...");
        while (!mqttClient.connect(mqtt_broker, mqtt_port))
        {
            Serial.print(".");
            delay(5000); // Wait 5 seconds before retrying
        }
        Serial.println("\nReconnected to MQTT broker");
    }

    // Read sensor data
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0F; // Convert to hPa
    int riverHeight = getDistance();              // Measure river height in mm
    if (riverHeight == -1)
    {
        Serial.println("Failed to read river height");
    }

    // Prepare JSON payload including the river height
    StaticJsonDocument<256> doc;
    doc["temperature"] = temperature;
    doc["humidity"] = humidity;
    doc["pressure"] = pressure;
    doc["riverHeight_mm"] = riverHeight; // Include river height in millimeters

    String data;
    serializeJson(doc, data);

    Serial.println(data);

    // Publish data to MQTT topic
    mqttClient.beginMessage(mqtt_topic);
    mqttClient.print(data);
    mqttClient.endMessage();

    delay(10000); // Wait 10 seconds before next read
}
