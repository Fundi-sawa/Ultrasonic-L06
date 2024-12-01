#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Wi-Fi credentials
const char *ssid = "Redmi Note 13 Pro";           // Replace with your Wi-Fi SSID
const char *password = "hotspottys";   // Replace with your Wi-Fi password

// MQTT broker settings
const char *broker = "test.mosquitto.org";     // Mosquitto test broker
const char *mqttUsername = "";                // MQTT username (leave blank for public brokers)
const char *mqttPassword = "";                // MQTT password (leave blank for public brokers)
const char *topicNotification = "log/parameter";

WiFiClient espClient;       // Wi-Fi client for PubSubClient
PubSubClient mqttClient(espClient); // Initialize MQTT client

// Ultrasonic Sensor Constants and Pins
#define ULTRASONIC_RX_PIN 10 // RX pin for Ultrasonic sensor
#define ULTRASONIC_TX_PIN 9 // TX pin for Ultrasonic sensor
HardwareSerial ultrasonicSensor(1); // Use UART2 for the L06 sensor

#define START_BYTE 0xFF // As per L06 data protocol
#define PACKET_SIZE 10  // L06 sends 10 bytes per reading

void debugPrint(const char *message) {
    Serial.println(message);
}

void connectToWiFi() {
    debugPrint("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        debugPrint(".");
        delay(1000);
    }

    debugPrint("Wi-Fi connected.");
    debugPrint("IP address: ");
    Serial.println(WiFi.localIP());
}

void connectToMQTTBroker() {
    debugPrint("Connecting to MQTT broker...");
    mqttClient.setServer(broker, 1883);

    while (!mqttClient.connected()) {
        debugPrint("Attempting to connect to MQTT broker...");
        if (mqttClient.connect("ESP32Client", mqttUsername, mqttPassword)) {
            debugPrint("Connected to MQTT broker.");
        } else {
            Serial.print("MQTT connection failed, rc=");
            Serial.println(mqttClient.state());
            debugPrint("Retrying in 5 seconds...");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200); // Initialize Serial Monitor
    ultrasonicSensor.begin(115200, SERIAL_8N1, ULTRASONIC_RX_PIN, ULTRASONIC_TX_PIN);

    debugPrint("Starting setup...");
    connectToWiFi();
    connectToMQTTBroker();
}

void loop() {
    // Ensure MQTT connection stays active
    if (!mqttClient.connected()) {
        debugPrint("MQTT disconnected! Reconnecting...");
        connectToMQTTBroker();
    }
    mqttClient.loop();

    // Publish a test message to the MQTT broker
    debugPrint("Publishing test message...");
    mqttClient.publish(topicNotification, "Hello, Mosquitto!");
    debugPrint("Message published.");

    // Handle Ultrasonic Sensor Data
    if (ultrasonicSensor.available() >= PACKET_SIZE) {
        uint8_t data[PACKET_SIZE];
        ultrasonicSensor.readBytes(data, PACKET_SIZE);

        // Verify start byte
        if (data[0] == START_BYTE) {
            int distance = (data[1] << 8) | data[2];
            float temperature = ((data[3] << 8) | data[4]) * 0.1;
            int signalStrength = (data[5] << 8) | data[6];
            float tiltAngle = ((data[7] << 8) | data[8]) * 0.1;

            // Format payload
            char payload[100];
            snprintf(payload, sizeof(payload), "{\"distance\":%d,\"temperature\":%.1f,\"signalStrength\":%d,\"tiltAngle\":%.1f}",
                     distance, temperature, signalStrength, tiltAngle);

            // Publish sensor data
            mqttClient.publish(topicNotification, payload);
            debugPrint("Sensor data published to MQTT.");
        } else {
            debugPrint("Invalid start byte.");
        }
    } else {
        debugPrint("Waiting for ultrasonic sensor data...");
    }

    delay(5000); // Publish every 5 seconds
}