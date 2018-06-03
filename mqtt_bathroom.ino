#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const char* WIFI_SSID = "***";
const char* WIFI_PASSWD = "***";

const byte RELAY_OUT1 = 12;
const byte RELAY_OUT2 = 13;

const byte MAX_WIFI_CONNECT_DELAY = 50;

const byte MOTION_SENSOR_PIN = 16;
boolean lastMotionSensorState = false;

IPAddress mqttServer(192, 168, 31, 100); // mosquitto address
const char* MQTT_USER = "smhome";
const char* MQTT_PASS = "smhome";
const char* MQTT_CLIENT = "ESP8266";

const char* MQTT_RELAY_TOPIC = "bathroom/relay";
const char* MQTT_MOTION_SENSOR_TOPIC = "bathroom/motion_sensor";
const char* MQTT_BRIGHTNESS_LOW_LEVEL = "{ \"value\" : \"low\" }";
const char* MQTT_BRIGHTNESS_HIGH_LEVEL = "{ \"value\" : \"high\" }";

WiFiUDP udp;
WiFiClient httpClient;

PubSubClient mqttClient(httpClient, mqttServer);

void publishMqtt(String pubTopic, String payload) {
  if (mqttClient.connected()) {
    mqttClient.publish(pubTopic, (char*) payload.c_str());
  }
}

void waitForWifiConnection() {
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && (retries < MAX_WIFI_CONNECT_DELAY)) {
    retries++;
    delay(500);
  }
  if (WiFi.status() != WL_CONNECTED) {
    abort();
  }
}

void motionSensorInit() {
  pinMode(MOTION_SENSOR_PIN, INPUT);
}

void relayInit() {
  pinMode(RELAY_OUT1, OUTPUT);
  pinMode(RELAY_OUT2, OUTPUT);
}

void readMotionSensor() {
  boolean sensorState = digitalRead(MOTION_SENSOR_PIN);
  if (sensorState != lastMotionSensorState) {
    lastMotionSensorState = sensorState;
    
    String payload = "{\"state\": ";
    payload += sensorState;
    payload += "}";
    publishMqtt(MQTT_MOTION_SENSOR_TOPIC, payload);
  }
}

void mQttCallback(const MQTT::Publish& pub) {
  if (pub.payload_len() == 0) {
    return;
  }
  String value = pub.payload_string();

  digitalWrite(RELAY_OUT1, LOW);
  digitalWrite(RELAY_OUT2, LOW);

  if (value == MQTT_BRIGHTNESS_LOW_LEVEL) {
    digitalWrite(RELAY_OUT1, HIGH);
  }
  else if (value == MQTT_BRIGHTNESS_HIGH_LEVEL) {
    digitalWrite(RELAY_OUT2, HIGH);
  }
}

void setup() {
  motionSensorInit();
  relayInit();
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);

  waitForWifiConnection();
}

void loop() {
  readMotionSensor();
  if (WiFi.status() != WL_CONNECTED) {
    abort();
    return;
  }  
  if (!mqttClient.connected()) {
    if (mqttClient.connect(MQTT::Connect(MQTT_CLIENT).set_auth(MQTT_USER, MQTT_PASS))) {
      Serial.println("Connected to MQTT");
      mqttClient.set_callback(mQttCallback);
      mqttClient.subscribe(MQTT_RELAY_TOPIC);
    }
  }
  if (mqttClient.connected()) {
    mqttClient.loop();
  }
  delay(100);
}
