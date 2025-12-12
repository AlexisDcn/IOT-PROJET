#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#ifndef WIFI_SSID
#define WIFI_SSID "iot"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "iotisis;"
#endif
#ifndef MQTT_SERVER
#define MQTT_SERVER "172.18.32.40"
#endif
#ifndef PATIENT_ID
#define PATIENT_ID "P001"
#endif

const char* ssid        = WIFI_SSID;
const char* password    = WIFI_PASS;
const char* mqtt_server = MQTT_SERVER;
const char* patient_id  = PATIENT_ID;

// Button
#define BUTTON_PIN 14
bool last_button_state = HIGH;
unsigned long last_debounce_time = 0;
#define DEBOUNCE_DELAY 50

// MQTT
WiFiClient espClient;
PubSubClient client(espClient);

void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("MQTT...");
    String clientId = String("ESP32_Button");
    if (client.connect(clientId.c_str())) {
      Serial.println("✓");
    } else {
      delay(5000);
    }
  }
}

void publish_button_press() {
  StaticJsonDocument<150> doc;
  doc["event"] = "button_pressed";
  doc["patient"] = patient_id;
  doc["timestamp"] = millis();
  
  char jsonBuffer[200];
  serializeJson(doc, jsonBuffer);
  
  char topic[100];
  snprintf(topic, sizeof(topic), "patient/%s/button/press", patient_id);
  
  if (client.publish(topic, jsonBuffer)) {
    Serial.println("✓ Button press published");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║  BUTTON POUSSOIR ESP32");
  Serial.println("╚════════════════════════════════════╝");
  
  // Button setup
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("✓ Button on GPIO14 (INPUT_PULLUP)");
  
  // WiFi
  Serial.println("WiFi...");
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println("");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("✓ WiFi OK");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  }
  
  // MQTT
  client.setServer(mqtt_server, 1883);
  
  Serial.println("\n════════════════════════════════════");
  Serial.println("Ready! Button monitoring @ GPIO14");
  Serial.println("════════════════════════════════════\n");
}

void loop() {
  // MQTT
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
  }
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();
  
  // Read button with debouncing
  bool button_state = digitalRead(BUTTON_PIN);
  
  // Debounce: wait 50ms to confirm state change
  if (button_state != last_button_state) {
    last_debounce_time = millis();
  }
  
  // If state stable for 50ms and it's a PRESS (HIGH → LOW)
  if ((millis() - last_debounce_time) > DEBOUNCE_DELAY) {
    if (button_state == LOW && last_button_state == HIGH) {
      // Button just pressed!
      Serial.println("🔘 BUTTON PRESSED!");
      publish_button_press();
    }
    last_button_state = button_state;
  }
  
  delay(10);
}
