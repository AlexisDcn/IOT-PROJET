#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Paramètres par défaut (écrasés par platformio.ini)
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

// Configuration LED (La LED intégrée sur FireBeetle32 est souvent sur le GPIO 2)
#define LED_PIN 5

const char* ssid        = WIFI_SSID;
const char* password    = WIFI_PASS;
const char* mqtt_server = MQTT_SERVER;

// Topic à écouter : patient/P001/test/status
char status_topic[100];

WiFiClient espClient;
PubSubClient client(espClient);

// --- Fonction appelée quand un message arrive ---
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message reçu [");
  Serial.print(topic);
  Serial.print("] : ");

  // Conversion du payload en String pour faciliter la comparaison
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  // LOGIQUE DE LA LED
  if (message == "START") {
    digitalWrite(LED_PIN, HIGH); // Allumer
    Serial.println("💡 LED ALLUMÉE (Test en cours)");
  }
  else if (message == "STOP") {
    digitalWrite(LED_PIN, LOW);  // Éteindre
    Serial.println("🌑 LED ÉTEINTE (Test fini)");
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connexion MQTT...");
    String clientId = "ESP32_LedStatus_" + String(PATIENT_ID);
    
    if (client.connect(clientId.c_str())) {
      Serial.println("Connecté !");
      // On s'abonne au topic de statut
      client.subscribe(status_topic);
      Serial.print("Abonné à : ");
      Serial.println(status_topic);
    } else {
      Serial.print("Echec, rc=");
      Serial.print(client.state());
      Serial.println(" nouvelle tentative dans 5s");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Eteint au démarrage

  // Construction du topic dynamique
  snprintf(status_topic, sizeof(status_topic), "patient/%s/test/status", PATIENT_ID);

  Serial.println("\n--- ESP32 TÉMOIN LUMINEUX ---");
 
  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi OK");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback); // On définit la fonction de réception
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}