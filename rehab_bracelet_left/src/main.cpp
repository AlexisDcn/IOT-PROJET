#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Edge Impulse
#include "rehabilitation-hemineglect_inferencing.h"

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
#ifndef HAND_SIDE
#define HAND_SIDE "left"
#endif

const char* ssid        = WIFI_SSID;
const char* password    = WIFI_PASS;
const char* mqtt_server = MQTT_SERVER;
const char* patient_id  = PATIENT_ID;
const char* hand_side   = HAND_SIDE;

// ⭐ CALIBRATION OFFSETS (mesurés au repos)
// Ajuste ces valeurs selon tes mesures réelles
#define OFFSET_X -1.0
#define OFFSET_Y 0.0
#define OFFSET_Z -8.50

// MPU6050
Adafruit_MPU6050 mpu;
#define FREQUENCY_HZ 62
#define INTERVAL_MS (1000 / (FREQUENCY_HZ + 1))
static unsigned long last_interval_ms = 0;

// Publication accéléromètre (toutes les 100ms = 10Hz)
#define ACCEL_PUBLISH_INTERVAL 100
// Décimation: publier toutes les N publications (1 = toutes les fois)
#ifndef ACCEL_PUBLISH_DECIMATION
#define ACCEL_PUBLISH_DECIMATION 4
#endif
static unsigned long last_accel_publish = 0;
static unsigned int accel_publish_counter = 0;

// Buffer for TinyML (625 samples = 10 sec @ 62.5 Hz, 3 axes)
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t feature_ix = 0;

// MQTT
WiFiClient espClient;
PubSubClient client(espClient);

void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("MQTT...");
    String clientId = String("ESP32_") + hand_side;
    if (client.connect(clientId.c_str())) {
      Serial.println("✓");
    } else {
      delay(5000);
    }
  }
}

void publish_accel(float x, float y, float z) {
  StaticJsonDocument<200> doc;
  doc["x"] = x;
  doc["y"] = y;
  doc["z"] = z;
  doc["hand"] = hand_side;
  doc["timestamp"] = millis();
  
  char jsonBuffer[256];
  serializeJson(doc, jsonBuffer);
  
  char topic[100];
  snprintf(topic, sizeof(topic), "patient/%s/bracelet/%s/accel", patient_id, hand_side);
  
  client.publish(topic, jsonBuffer);
}

void publish_activity(const char* activity, float confidence) {
  StaticJsonDocument<200> doc;
  doc["activity"] = activity;
  doc["confidence"] = confidence;
  doc["hand"] = hand_side;
  doc["timestamp"] = millis();
  
  char jsonBuffer[256];
  serializeJson(doc, jsonBuffer);
  
  char topic[100];
  snprintf(topic, sizeof(topic), "patient/%s/bracelet/%s/activity", patient_id, hand_side);
  
  if (client.publish(topic, jsonBuffer)) {
    Serial.print("✓ ");
    Serial.print(activity);
    Serial.print(" (");
    Serial.print(confidence);
    Serial.println(")");
  }
}

void ei_printf(const char *format, ...) {
  static char print_buf[1024] = { 0 };
  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);
  if (r > 0) {
    Serial.write(print_buf);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║  BRACELET GAUCHE + MPU6050");
  Serial.println("║  Accel Calibré + Activity MQTT");
  Serial.println("╚════════════════════════════════════╝");
  
  // MPU6050
  Serial.println("Init MPU6050...");
  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 NOT FOUND!");
    Serial.println("Check: GPIO21=SDA, GPIO22=SCL");
    while (1) delay(10);
  }
  Serial.println("✓ MPU6050 OK");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
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
  client.setBufferSize(512);
  
  // TinyML Info
  Serial.print("TinyML model: ");
  Serial.println(EI_CLASSIFIER_PROJECT_NAME);
  Serial.print("Calibration: X=");
  Serial.print(OFFSET_X);
  Serial.print(" Y=");
  Serial.print(OFFSET_Y);
  Serial.print(" Z=");
  Serial.println(OFFSET_Z);
  
  Serial.println("\n════════════════════════════════════");
  Serial.println("Ready! Recording @ 62.5 Hz...");
  Serial.println("Publishing accel @ 10 Hz");
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
  
  // Read MPU @ 62.5 Hz and buffer features
  if (millis() > last_interval_ms + INTERVAL_MS) {
    last_interval_ms = millis();
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // ⭐ CALIBRATION: Soustraire les offsets mesurés au repos
    float cal_x = a.acceleration.x - OFFSET_X;
    float cal_y = a.acceleration.y - OFFSET_Y;
    float cal_z = a.acceleration.z - OFFSET_Z;
    
    // Store calibrated data for TinyML
    features[feature_ix++] = cal_x;
    features[feature_ix++] = cal_y;
    features[feature_ix++] = cal_z;
    
    // 📡 Publish accelerometer data @ configured interval, with decimation
    if (millis() > last_accel_publish + ACCEL_PUBLISH_INTERVAL) {
      last_accel_publish = millis();
      accel_publish_counter++;
      if (accel_publish_counter >= ACCEL_PUBLISH_DECIMATION) {
        accel_publish_counter = 0;
        publish_accel(cal_x, cal_y, cal_z);
      }
    }
    
    // When buffer full, run inference
    if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
      ei_printf("\n🔍 Inference...\n");
      
      signal_t signal;
      ei_impulse_result_t result;
      
      int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
      if (err != 0) {
        ei_printf("❌ Signal error (%d)\n", err);
        feature_ix = 0;
        return;
      }
      
      // Run classifier
      EI_IMPULSE_ERROR res = run_classifier(&signal, &result, true);
      if (res != 0) {
        ei_printf("❌ Classifier error\n");
        feature_ix = 0;
        return;
      }
      
      // Find best prediction
      float max_confidence = 0;
      const char* best_activity = "unknown";
      
      ei_printf("Results:\n");
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("  %s: %.2f%%\n", 
          result.classification[ix].label, 
          result.classification[ix].value * 100);
        
        if (result.classification[ix].value > max_confidence) {
          max_confidence = result.classification[ix].value;
          best_activity = result.classification[ix].label;
        }
      }
      
      ei_printf("Inference time: %lu ms\n", result.timing.classification);
      
      if (max_confidence > 0.5) {
        ei_printf("✓ Best: %s (%.0f%%)\n", best_activity, max_confidence * 100);
        publish_activity(best_activity, max_confidence);
      } else {
        ei_printf("⚠ Low confidence (%.0f%%)\n", max_confidence * 100);
      }
      
      feature_ix = 0;
    }
  }
}