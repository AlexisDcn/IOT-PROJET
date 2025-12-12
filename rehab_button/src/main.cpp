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

// Button (entre GPIO14 et GND)
#define BUTTON_PIN 14
unsigned long last_press_time = 0;
#define DEBOUNCE_DELAY 200  // ms
bool last_state = HIGH;     // avec INPUT_PULLUP, au repos = HIGH

// MQTT
WiFiClient espClient;
PubSubClient client(espClient);

void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("MQTT...");
    String clientId = String("ESP32_Button_") + random(10000);
    if (client.connect(clientId.c_str())) {
      Serial.println("✓");
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 5s");
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
    Serial.print("✓ MQTT published: ");
    Serial.println(jsonBuffer);
  } else {
    Serial.println("❌ MQTT publish failed");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║  BUTTON POUSSOIR ESP32");
  Serial.println("╚════════════════════════════════════╝");

  // Bouton
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  delay(100); // Stabilisation
  last_state = digitalRead(BUTTON_PIN); // Sync avec état réel
  Serial.println("✓ Button on GPIO14 (INPUT_PULLUP)");
  Serial.print("Initial state: ");
  Serial.println(last_state ? "HIGH" : "LOW");

  // WiFi
  Serial.println("\nWiFi connecting...");
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println("");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("✓ WiFi OK");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("❌ WiFi failed!");
  }

  // MQTT
  client.setServer(mqtt_server, 1883);
  reconnect_mqtt();

  Serial.println("\n════════════════════════════════════");
  Serial.println("Ready! Press button on GPIO14...");
  Serial.println("════════════════════════════════════\n");
}

void loop() {
  // MQTT maintenance
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
  }
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();

  // Lecture bouton
  bool current = digitalRead(BUTTON_PIN);

  // Debug: afficher les changements d'état
  if (current != last_state) {
    Serial.print("State change: ");
    Serial.print(last_state ? "HIGH" : "LOW");
    Serial.print(" -> ");
    Serial.println(current ? "HIGH" : "LOW");
  }

  // Front descendant: HIGH -> LOW (appui avec INPUT_PULLUP)
  if (last_state == HIGH && current == LOW) {
    unsigned long now = millis();
    if (now - last_press_time > DEBOUNCE_DELAY) {
      Serial.println("🔘 BUTTON PRESSED (edge)!");
      publish_button_press();
      last_press_time = now;
    } else {
      Serial.println("(debounced - too soon)");
    }
  }

  last_state = current;
  delay(5);
}
