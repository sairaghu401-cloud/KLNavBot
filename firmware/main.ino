/**
 * KLNavBot — ESP32 Main Firmware
 * Connects to WiFi, Firebase Realtime Database,
 * reads ultrasonic obstacle sensor, controls motors,
 * and streams simulated GPS coordinates.
 *
 * Hardware:
 *   - ESP32 Dev Module
 *   - HC-SR04 Ultrasonic Sensor (TRIG=5, ECHO=18)
 *   - L298N Motor Driver (IN1=26, IN2=27, IN3=14, IN4=12)
 *   - ENA=25 (PWM), ENB=13 (PWM)
 *   - NEO-6M GPS module (RX=16, TX=17) — optional
 *
 * Libraries required (install via Arduino Library Manager):
 *   - Firebase ESP Client  : mobizt/Firebase ESP Client
 *   - ArduinoJson          : bblanchon/ArduinoJson
 *
 * Author : KLNavBot Team
 * Version: 1.0.0
 */

#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>      // mobizt/Firebase ESP Client
#include <ArduinoJson.h>
#include "motor_control.h"

// ── WiFi Credentials ────────────────────────────────────────
#define WIFI_SSID     "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// ── Firebase Credentials ─────────────────────────────────────
#define FIREBASE_HOST "klu-robot-tracker-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "YOUR_FIREBASE_DATABASE_SECRET"   // or use token

// ── HC-SR04 Ultrasonic Pins ──────────────────────────────────
#define TRIG_PIN 5
#define ECHO_PIN 18
#define OBSTACLE_THRESHOLD_CM 30   // halt if obstacle within 30 cm

// ── Timing ───────────────────────────────────────────────────
#define SENSOR_INTERVAL_MS   100   // obstacle poll interval
#define FIREBASE_INTERVAL_MS 500   // Firebase push interval
#define GPS_INTERVAL_MS      2000  // GPS push interval

// ── Global Objects ───────────────────────────────────────────
FirebaseData  fbData;
FirebaseAuth  fbAuth;
FirebaseConfig fbConfig;

// ── State ────────────────────────────────────────────────────
String  lastCommand       = "stop";
bool    obstacleDetected  = false;
float   simLat            = 16.441589f;  // GPS simulation start
float   simLon            = 80.624234f;
unsigned long lastSensorMs   = 0;
unsigned long lastFirebaseMs = 0;
unsigned long lastGpsMs      = 0;

// ─────────────────────────────────────────────────────────────
void connectWiFi() {
  Serial.print("[WiFi] Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint8_t attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print('.');
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected — IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[WiFi] FAILED — restarting in 5s");
    delay(5000);
    ESP.restart();
  }
}

// ─────────────────────────────────────────────────────────────
void connectFirebase() {
  fbConfig.host        = FIREBASE_HOST;
  fbConfig.signer.tokens.legacy_token = FIREBASE_AUTH;

  Firebase.begin(&fbConfig, &fbAuth);
  Firebase.reconnectWiFi(true);
  Firebase.setDoubleDecimalPlaces(6);

  Serial.println("[Firebase] Initialised");
}

// ─────────────────────────────────────────────────────────────
/** Read ultrasonic distance in centimetres */
float readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30 ms timeout
  if (duration == 0) return 999.0f;                 // no echo → clear
  return (duration * 0.0343f) / 2.0f;
}

// ─────────────────────────────────────────────────────────────
/** Read latest movement command from Firebase */
void readCommandFromFirebase() {
  if (Firebase.getString(fbData, "/robotCommands/move")) {
    String cmd = fbData.stringData();
    if (cmd != lastCommand) {
      lastCommand = cmd;
      Serial.println("[CMD] New command: " + cmd);
    }
  }
}

// ─────────────────────────────────────────────────────────────
/** Push robot status and GPS to Firebase */
void pushStatusToFirebase() {
  // Build JSON payload for robotStatus
  StaticJsonDocument<128> statusDoc;
  statusDoc["obstacle"] = obstacleDetected;
  statusDoc["state"]    = lastCommand;
  statusDoc["battery"]  = 85;   // placeholder — wire ADC for real value

  String statusJson;
  serializeJson(statusDoc, statusJson);
  Firebase.setString(fbData, "/robotStatus", statusJson);
}

void pushGPSToFirebase() {
  // Simulate slight GPS drift for demo
  simLat += ((random(-5, 5)) * 0.000001f);
  simLon += ((random(-5, 5)) * 0.000001f);

  FirebaseJson json;
  json.set("lat", simLat);
  json.set("lon", simLon);
  json.set("ts",  millis());
  Firebase.setJSON(fbData, "/robotGPS", json);
}

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== KLNavBot Firmware v1.0.0 ===");

  // Ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Motor controller
  motorSetup();

  // Network + Firebase
  connectWiFi();
  connectFirebase();

  // Initial stop command
  motorStop();
  Serial.println("[INIT] Ready");
}

// ─────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── 1. Obstacle detection ─────────────────────────────────
  if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;

    float dist = readUltrasonicCm();
    obstacleDetected = (dist < OBSTACLE_THRESHOLD_CM);

    if (obstacleDetected) {
      motorStop();
      Serial.printf("[SENSOR] Obstacle at %.1f cm — HALT\n", dist);
    }
  }

  // ── 2. Execute motion command (only if path is clear) ─────
  if (!obstacleDetected) {
    if      (lastCommand == "forward")  motorForward(200);
    else if (lastCommand == "backward") motorBackward(200);
    else if (lastCommand == "left")     motorLeft(180);
    else if (lastCommand == "right")    motorRight(180);
    else                                motorStop();
  }

  // ── 3. Firebase read/write ────────────────────────────────
  if (now - lastFirebaseMs >= FIREBASE_INTERVAL_MS) {
    lastFirebaseMs = now;
    readCommandFromFirebase();
    pushStatusToFirebase();
  }

  // ── 4. GPS push ───────────────────────────────────────────
  if (now - lastGpsMs >= GPS_INTERVAL_MS) {
    lastGpsMs = now;
    pushGPSToFirebase();
  }
}
