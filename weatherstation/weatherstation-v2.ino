#include <MKRNB.h>
#include <RTCZero.h>
#include <ArduinoLowPower.h>
#include <ArduinoMqttClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

// Ultrasonic sensor JSN-SR04T/AJ-SR04M
#define TRIG_PIN 5
#define ECHO_PIN 6
#define rxPin 10
#define txPin 11

// Low power sleep duration - How long the unit will sleep between waking up to take sensor readings
const unsigned long LowPowerSleepDuration = 900000; // 15 minutes in milliseconds

// Real-time clock
RTCZero rtc;
unsigned long lastSyncTime = 0;
const unsigned long syncInterval = 604800; // 1 week in seconds // How often should the device to sync time from network

// Cellular credentials
const char PINNUMBER[] = "";                // Pin number if SIM is PIN protected
const char GPRS_APN[] = "telstra.internet"; // Telstra APN
const char GPRS_LOGIN[] = "";               // Leave blank if not used
const char GPRS_PASSWORD[] = "";            // Leave blank if not used

// MQTT Broker settings
const char mqtt_broker[] = "mqtt_broker_address";
const int mqtt_port = 1883;
const char mqtt_topic[] = "sensor/weatherstation";
const String clientId = "WeatherStation-" + String(random(0xffff), HEX);

// Initialize the JSN-SR04T sensor
SoftwareSerial jsnSerial(rxPin, txPin);

// Initialize the BME280 sensor
Adafruit_BME280 bme; // I2C

// Initialize the cellular hardware
NB nbAccess;
GPRS gprs;
NBClient nbClient;
MqttClient mqttClient(nbClient);

// For accumulating data payload
String accumulatedData = ""; // Ensure that the accumulated data does not exceed your MQTT broker's message size limits. Consider more efficient data structures or storage mechanisms (like a circular buffer) to handle data accumulation and transmission, especially to avoid potential memory fragmentation issues associated with String concatenation.
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 3600; // 1 hour in seconds

int getDistance();
bool updateTimeFromNetwork();
String parseNetworkTime(const String &response); // Placeholder for actual implementation

bool initializeBME280()
{
    if (!bme.begin())
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        return false; // Initialization failed
    }
    return true; // Successfully initialized
}

void setup()
{
    Serial.begin(115200);

    // Initialize RTC
    rtc.begin();

    // Ultrasonic sensor
    jsnSerial.begin(9600);
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW); // Ensure trigger pin is low for ultrasonic sensor
    pinMode(ECHO_PIN, INPUT);

    // Start the BME280 sensor
    Serial.println("Initialize BME280: " + initializeBME280());

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
    mqttClient.setId(clientId.c_str());
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

bool updateTimeFromNetwork()
{
    NBModem modem;

    Serial.println("Updating time from network...");
    modem.begin();

    // Send an AT command to get the network time
    // The response handling and actual time parsing logic will depend on your network and modem
    // See https://github.com/arduino-libraries/MKRNB/blob/master/examples/GPRSUdpNtpClient/GPRSUdpNtpClient.ino
    String response;
    bool success = modem.sendATCommand("AT+CCLK?", response); // This is a simplified placeholder
    if (success)
    {
        String networkTime = parseNetworkTime(response);
        if (networkTime != "")
        {
            // Convert and set time here
            Serial.println("Network time updated");
            return true;
        }
    }
    Serial.println("Failed to fetch network time");
    return false;
}

String parseNetworkTime(const String &response)
{
    // Placeholder: Will need to implement this based on the actual format of your network's time response
    return "";
}

void loop()
{
    unsigned long currentTime = rtc.getEpoch();

    // Sync time every week
    if (currentTime - lastSyncTime > syncInterval)
    {
        if (updateTimeFromNetwork())
        {
            lastSyncTime = rtc.getEpoch();
        }
    }

    // Accumulate sensor readings every 15 minutes
    if (currentTime % 900 == 0)
    { // Every 15 minutes

        // Read sensor data
        float temperature = bme.readTemperature();
        float humidity = bme.readHumidity();
        float pressure = bme.readPressure() / 100.0F; // Convert to hPa

        if (isnan(temperature) || isnan(humidity) || isnan(pressure))
        {
            Serial.println("Error reading from BME280 sensor!");
            initializeBME280(); // Attempt to reinitialize the sensor
            delay(5000);        // Delay between retries
            return;             // Skip this iteration if BME280 sensor reading failed
        }

        int riverHeight = getDistance(); // Measure river height in mm
        if (riverHeight == -1)
        {
            Serial.println("Failed to read river height from JSN-SR04T/AJ-SR04M ultrasonic sensor!");
            delay(5000); // Delay between retries
            return;      // Skip this iteration if river height reading failed
        }

        // Prepare JSON payload including the river height
        StaticJsonDocument<256> doc;
        doc["timestamp"] = currentTime; // Include timestamp in epoch format [seconds since 1970-01-01T00:00:00Z]
        doc["temperature"] = temperature;
        doc["humidity"] = humidity;
        doc["pressure"] = pressure;
        doc["riverHeight_mm"] = riverHeight; // Include river height in millimeters

        String data;
        serializeJson(doc, data);

        // Append to accumulatedData
        accumulatedData += data + "\n";

        // Print data to serial monitor
        Serial.println(data);
    }

    // Data publication logic - Publish accumulated data every hour
    if (currentTime - lastPublishTime >= publishInterval && !accumulatedData.isEmpty())
    {
        if (!mqttClient.connected())
        {
            Serial.print("Reconnecting to MQTT broker...");
            while (!mqttClient.connect(mqtt_broker, mqtt_port, clientId.c_str(), "mqtt_username", "mqtt_password"))
            {
                Serial.print(".");
                delay(5000); // Delay between retries
            }
            Serial.println("\nReconnected to MQTT broker");
        }
        mqttClient.beginMessage(mqtt_topic);
        mqttClient.print(accumulatedData);
        mqttClient.endMessage();
        accumulatedData = ""; // Clear the accumulated data after publishing
        lastPublishTime = currentTime;
    }
    // Enter low power mode until next 15-minute mark
    LowPower.sleep(LowPowerSleepDuration - (millis() % LowPowerSleepDuration)); // Consider any potential drift in timekeeping, especially if the internal RTC is not periodically synchronized with a more accurate time source
}
