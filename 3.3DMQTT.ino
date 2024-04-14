#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>

// WiFi settings
const char* ssid = "SHIVJI"; // Replace with your WiFi SSID
const char* password = "Dreams11"; // Replace with your WiFi password

// MQTT Broker settings
const char* broker = "broker.emqx.io";
const int port = 1883;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// Pins
const int trigPin = 7; // Pin for ultrasonic sensor trigger
const int echoPin = 6; // Pin for ultrasonic sensor echo
const int ledPin = 5;  // Pin for LED

void connectingtoWiFi() {
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("Connected!");
}

void onMessage(int messageSize) {
    String topic = mqttClient.messageTopic();
    String payload = "";
    
    while (mqttClient.available()) {
        payload += (char)mqttClient.read();
    }

    Serial.println("Received message: '" + payload + "' on topic: '" + topic + "'");

    if (topic == "SIT210/wave") {
        // Flash the LED three times for a wave
        for (int i = 0; i < 3; i++) {
            digitalWrite(ledPin, HIGH);
            delay(500);
            digitalWrite(ledPin, LOW);
            delay(500);
        }
    } else if (topic == "SIT210/pat") {
        // Flash the LED once for a pat
        digitalWrite(ledPin, HIGH);
        delay(300);
        digitalWrite(ledPin, LOW);
    }
}

void connectingtoMQTT() {
    Serial.print("Connecting to MQTT");
    while (!mqttClient.connect(broker, port)) {
        Serial.print(".");
        delay(5000);
    }

    Serial.println("Connected!");
    mqttClient.subscribe("SIT210/wave"); // Subscribing to the set channel to give data
    mqttClient.subscribe("SIT210/pat"); // Subscribing to the set channel to give data
}

void setup() {
    Serial.begin(9600);
    connectingtoWiFi();
    connectingtoMQTT();

    pinMode(trigPin, OUTPUT); //Pin for wave output
    pinMode(echoPin, INPUT); //Pin for echo input
    pinMode(ledPin, OUTPUT); //Pin for led output
}

void loop() {
    static unsigned long lastDetectionTime = 0;
    const unsigned long detectionInterval = 2000; // Delay between detections in milliseconds

    if (!mqttClient.connected()) {
        connectingtoMQTT();
    }

    mqttClient.poll();

    long duration = 0, distance = 0;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = (duration / 2) / 29.1; // Calculating the distance based on the time of echo

    unsigned long currentTime = millis();
    if (currentTime - lastDetectionTime > detectionInterval) {
        if (distance < 20) {
            Serial.println("Wave detected");
            mqttClient.beginMessage("SIT210/wave");
            mqttClient.print("Wave from Vidul");
            mqttClient.endMessage();
            lastDetectionTime = currentTime; // Updating the last detection time
        } else if (distance >= 20 && distance <= 30) {
            Serial.println("Pat detected");
            mqttClient.beginMessage("SIT210/pat");
            mqttClient.print("Pat from Vidul");
            mqttClient.endMessage();
            lastDetectionTime = currentTime; // Updating the last detection time
        }
    }
}