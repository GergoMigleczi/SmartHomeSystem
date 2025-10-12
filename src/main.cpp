#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include "secrets.h"

// ==== OLED config ====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ==== Pin config ====
const int pirPin    = 2;
const int flamePin  = 4;
const int ledPin    = 5;
const int tempPin   = 15;
const int CameraPin_RX = 16;  // RX2 - receives from CAM TX
const int CameraPin_TX = 17;  // TX2 - sends to CAM RX
//const int OledSDA = 21;
//const int OledSCK = 22;
const int buzzerPin = 18;
const int gasPin    = 34;

// ==== Thresholds ====
const float TEMP_MIN = 6.0;
const float TEMP_MAX = 30.0;
const int GAS_THRESHOLD = 3000;

// ==== Wi-Fi & Telegram ====
WiFiClientSecure client;
UniversalTelegramBot bot(TELEGRAM_TOKEN, client);

// ==== Temperature setup ====
OneWire oneWire(tempPin);
DallasTemperature tempSensor(&oneWire);

// ==== UART for Camera ====
HardwareSerial CameraSerial(2); // Use UART2
const char CAPTURE_CMD = 'C';   // Command to trigger capture

// ==== Global state ====
struct SensorData {
  int gasValue;
  bool motionDetected;
  bool flameDetected;
  float temperature;
};

bool gasAlertState = false;
bool motionAlertState = false;
bool flameAlertState = false;
bool tempAlertState = false;
bool imageRequested = false; // Track if image was already requested

// -----------------------------------------------------
// Function declarations
// -----------------------------------------------------
void initializeSystem();
SensorData readSensors();
bool checkForAlerts(const SensorData &data);
void logToSerial(const SensorData &data, bool alert);
void updateDisplay(const SensorData &data, bool alert);
void handleAlert(bool alert);
void handleTelegramNotifications(const SensorData &data);
void handleCameraCapture();

// -----------------------------------------------------
// Setup
// -----------------------------------------------------
void setup() {
  Serial.begin(115200);
  initializeSystem();
}

// -----------------------------------------------------
// Main loop
// -----------------------------------------------------
void loop() {
  SensorData data = readSensors();
  bool alert = checkForAlerts(data);

  logToSerial(data, alert);
  updateDisplay(data, alert);
  handleAlert(alert);
  handleCameraCapture(); // Check if camera capture needed
  handleTelegramNotifications(data);

  delay(500);
}

// -----------------------------------------------------
// Function definitions
// -----------------------------------------------------
void initializeSystem() {
  pinMode(pirPin, INPUT);
  pinMode(flamePin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  tempSensor.begin();

  // Initialize Camera UART
  CameraSerial.begin(115200, SERIAL_8N1, CameraPin_RX, CameraPin_TX);
  Serial.println("Camera UART initialized");

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("System Initializing..."));
  display.display();

  // Wi-Fi connection
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");

  client.setInsecure(); // skip certificate check for Telegram
  bot.sendMessage(TELEGRAM_CHAT_ID, "ESP32 Safety System is online!", "");

  delay(2000);
}

SensorData readSensors() {
  SensorData data;
  data.gasValue = analogRead(gasPin);
  data.motionDetected = digitalRead(pirPin);
  data.flameDetected = digitalRead(flamePin);
  tempSensor.requestTemperatures();
  data.temperature = tempSensor.getTempCByIndex(0);
  return data;
}

bool checkForAlerts(const SensorData &data) {
  bool gasAlert = data.gasValue > GAS_THRESHOLD;
  bool tempAlert = (data.temperature < TEMP_MIN || data.temperature > TEMP_MAX);
  return gasAlert || data.motionDetected || data.flameDetected || tempAlert;
}

void logToSerial(const SensorData &data, bool alert) {
  Serial.print("Gas: ");
  Serial.print(data.gasValue);
  Serial.print(" | Motion: ");
  Serial.print(data.motionDetected ? "YES" : "NO");
  Serial.print(" | Flame: ");
  Serial.print(data.flameDetected ? "YES" : "NO");
  Serial.print(" | Temp: ");
  Serial.print(data.temperature);
  Serial.print("°C");
  if (alert) Serial.print(" [ALERT]");
  Serial.println();
}

void updateDisplay(const SensorData &data, bool alert) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  display.print("Gas: ");
  display.println(data.gasValue > GAS_THRESHOLD ? "ALERT" : "OK");

  display.print("Motion: ");
  display.println(data.motionDetected ? "YES" : "NO");

  display.print("Flame: ");
  display.println(data.flameDetected ? "YES" : "NO");

  display.print("Temp: ");
  display.print(data.temperature, 1);
  display.println("C");

  display.setTextSize(2);
  display.println(alert ? "ALERT!" : "Status OK");
  display.display();
}

void handleAlert(bool alert) {
  digitalWrite(ledPin, alert ? HIGH : LOW);
  digitalWrite(buzzerPin, alert ? HIGH : LOW);
}

// -----------------------------------------------------
// Camera capture logic
// -----------------------------------------------------
void handleCameraCapture() {
  // Check if flame OR motion is detected
  bool shouldCapture = flameAlertState || motionAlertState;
  
  // Only send capture command if:
  // 1. Flame or motion is currently detected
  // 2. Image hasn't been requested yet for this alert
  if (shouldCapture && !imageRequested) {
    CameraSerial.write(CAPTURE_CMD);
    Serial.println("📷 Capture command sent to camera!");
    imageRequested = true;
  }
  
  // Reset imageRequested flag when both alerts are cleared
  if (!flameAlertState && !motionAlertState) {
    imageRequested = false;
  }
}

// -----------------------------------------------------
// Telegram notification logic
// -----------------------------------------------------
void handleTelegramNotifications(const SensorData &data) {
  bool gasAlert = data.gasValue > GAS_THRESHOLD;
  bool motionAlert = data.motionDetected;
  bool flameAlert = data.flameDetected;
  bool tempAlert = (data.temperature < TEMP_MIN || data.temperature > TEMP_MAX);

  String message = "";

  if (gasAlert && !gasAlertState) {
    message += "⚠️ Gas alert detected!\n";
  }
  if (motionAlert && !motionAlertState) {
    message += "⚠️ Motion detected!\n";
  }
  if (flameAlert && !flameAlertState) {
    message += "🔥 Flame detected!\n";
  }
  if (tempAlert && !tempAlertState) {
    message += "🌡️ Temperature out of range: " + String(data.temperature, 1) + "°C\n";
  }

  if (message.length() > 0) {
    bot.sendMessage(TELEGRAM_CHAT_ID, message, "");
  }

  // Update previous states
  gasAlertState = gasAlert;
  motionAlertState = motionAlert;
  flameAlertState = flameAlert;
  tempAlertState = tempAlert;
}