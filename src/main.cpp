#include <Arduino.h>
#include <Wire.h>
#include <oled.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ThingSpeak.h>
#include "secrets.h"

// ==== Debugging ====
const bool DEBUG = true;

// ==== OLED config ====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

OLED display = OLED (
  21,                  // sda pin for I2C comunication
  22,                  // scl pin for I2C comunication
  NO_RESET_PIN,        // Reset pin (default: none)
  OLED::W_128,         // Display width, must be one of enum: W_96 or W_128 (default: W_128).
  OLED::H_64,          // Display height, must be one of enum: H_16, H_32 or OLED::H_64 (default: H_32).
  OLED::CTRL_SH1106,   // Display controller chip, must be one of enum: CTRL_SH1106 or CTRL_SSD1306 (default: CTRL_SSD1306).                  
  0x3C                 // I2C address (default: 0x3C)
);

// ==== Pin config ====
const int pirPin    = 2;
const int flamePin  = 4;
const int ledPin    = 5;
const int tempPin   = 15;
const int CameraPin_RX = 16;  // RX2 - receives from CAM TX
const int CameraPin_TX = 17;  // TX2 - sends to CAM RX
const int buzzerPin = 18;
const int gasPin    = 34;

// ==== Thresholds ====
const float TEMP_MIN = 6.0;
const float TEMP_MAX = 30.0;
const int GAS_THRESHOLD = 3000;

// ==== Wi-Fi ====
WiFiClientSecure client;

// ==== Telegram ====
const char* telegramHost = "api.telegram.org"; // Telegram API server host
UniversalTelegramBot bot(TELEGRAM_TOKEN, client);

// ==== Telegram Commands Setup ====
unsigned long lastTelegramCheck = 0;
const unsigned long TELEGRAM_INTERVAL = 2000; // check every 2 seconds

bool telegramCommandTakePicture = false;
bool telegramCommandSendTemp = false;

// ===== ThingSpeak Cooldown =====
const unsigned long COOLDOWN = 30000; // 30 seconds - avoid slowing system down by sending readings too frequently

// ===== ThingSpeak Fields =====
int FIELD_GAS = 1;
int FIELD_TEMPERATURE = 2;
int FIELD_FLAME = 3;
int FIELD_MOTION = 4;

// ==== Temperature setup ====
OneWire oneWire(tempPin);
DallasTemperature tempSensor(&oneWire);

// ==== UART for Camera ====
HardwareSerial CameraSerial(2); // Use UART2
const int UART_BAUD_RATE = 115200;
const char CAPTURE_CMD = 'C';   // Command to trigger capture
const char RESPONSE_START = 'S'; // Start of image transmission
const char RESPONSE_END = 'E';   // End of image transmission
const char RESPONSE_ERROR = 'X'; // Error during capture
const char RESPONSE_ACK = 'A';  // Acknowledgment
const char BLINK_CMD = 'B'; // Command to blink LED

// ==== Camera State Machine ====
enum CameraState {
  CAM_IDLE,              // Not capturing
  CAM_WAITING_FOR_IMAGE, // Sent 'C', waiting for data
  CAM_IMAGE_READY        // Image received, ready to send
};

CameraState cameraState = CAM_IDLE;

// ==== Image Data Structure ====
struct ImageData {
  uint8_t* buffer;      // Image bytes
  uint32_t size;        // Image size
  uint32_t bytesRead;   // Bytes received so far
  float progress;      // Progress percentage
  bool available;       // Is image ready to send?
};

ImageData capturedImage = {NULL, 0, 0, 0.0, false};

// ==== Sensor Data Structure ====
struct SensorData {
  int gasValue;
  bool motionDetected;
  bool flameDetected;
  float temperature;
};

SensorData currentSensorData {0, false, false, 0.0};

// ==== ThingSpeak Sensor Data Structure ====
template <typename T>
struct Field {
  T value;
  bool frozen = false;
};

struct ThingSpeakSensorData {
  Field<int> gasValue;
  Field<bool> motionDetected;
  Field<bool> flameDetected;
  Field<float> temperature;
};
ThingSpeakSensorData thingSpeakSensorData;

// ==== Alert State Management Structure ====
struct AlertState {
  bool current;
  bool previous;
  
  void update(bool condition) {
    previous = current;
    current = condition;
  }
  
  bool isRisingEdge() const {
    return current && !previous;
  }
  
  bool isActive() const {
    return current;
  }
};

// ==== Alert States ====
AlertState gasAlert;
AlertState motionAlert;
AlertState flameAlert;
AlertState tempAlert;
AlertState systemAlert;

// -----------------------------------------------------
// Function declarations
// -----------------------------------------------------
void initializeSystem();
SensorData readSensors();
bool checkForAlerts();
void updateAllAlertStates();
void logSensorData();
void updateDisplay();
void handleLocalAlert();

void updateThingSpeakSensorData();
void sendToThingSpeak();

void manageCameraCapture();
bool shouldRequestCapture();
void sendCommandToCamera(char command);
bool tryReceiveImageData();
bool isReceivingImage();

void handleTelegramCommands();
void handleTelegramNotifications();
void checkAndSendAlertMessages();
void sendSensorDataIfRequested();
void sendCapturedImageIfReady();
void sendPhotoToTelegram(uint8_t* buffer, size_t length);

void freeImageBuffer(ImageData &image);

void logDebug(const char* message);
void logDebugf(const char* format, ...);

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
  // Read sensors
  currentSensorData = readSensors();

  // Update global alert states once
  updateAllAlertStates();  

  //logSensorData();
  updateDisplay();
  handleLocalAlert();
  
  // Handle Telegram commands
  handleTelegramCommands();

  // Handle all camera logic via state machine
  manageCameraCapture();  
  
  // Handle Telegram notifications
  handleTelegramNotifications();

  // Send sensor data to ThingSpeak
  updateThingSpeakSensorData();
  sendToThingSpeak();
  
  delay(500);
}

// -----------------------------------------------------
// System Initialization
// -----------------------------------------------------
void initializeSystem() {
  pinMode(pirPin, INPUT);
  pinMode(flamePin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  tempSensor.begin();

  // Initialize Camera UART
  CameraSerial.setRxBufferSize(1024);  // increase RX buffer size
  CameraSerial.setTxBufferSize(1024);  // increase TX buffer size
  CameraSerial.begin(UART_BAUD_RATE, SERIAL_8N1, CameraPin_RX, CameraPin_TX);
  logDebug("Camera UART initialized");
  sendCommandToCamera(BLINK_CMD); // Blink camera LED to indicate ready

  // Initialize OLED
  display.begin();
  display.clear();
  display.noInverse();
  display.drawString(2,1,"System Initialising...");
  display.display();
  

  // Wi-Fi connection
  logDebug("Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  display.drawString(2,2,"Connecting to Wifi...");
  display.display();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  logDebug("Connected!");

  client.setInsecure(); // skip certificate check
  bot.sendMessage(TELEGRAM_CHAT_ID, "ESP32 Safety System is online!", "");

  // Initialise ThingSpeak
  ThingSpeak.begin(client);
  logDebug("ThingSpeak initialised");

  delay(2000);
}

// -----------------------------------------------------
// Sensor Reading
// -----------------------------------------------------
SensorData readSensors() {
  SensorData data;
  data.gasValue = analogRead(gasPin);
  data.motionDetected = digitalRead(pirPin);
  data.flameDetected = (digitalRead(flamePin) == 1 ? false : true);
  tempSensor.requestTemperatures();
  data.temperature = tempSensor.getTempCByIndex(0);
  return data;
}

// -----------------------------------------------------
// Check for any alerts based on sensor data
// -----------------------------------------------------
bool checkForAlerts() {
  return gasAlert.isActive() || motionAlert.isActive() || 
         flameAlert.isActive() || tempAlert.isActive();
}

// -----------------------------------------------------
// Update global alert states from sensor data
// -----------------------------------------------------
void updateAllAlertStates() {
  gasAlert.update(currentSensorData.gasValue > GAS_THRESHOLD);
  motionAlert.update(currentSensorData.motionDetected);
  flameAlert.update(currentSensorData.flameDetected);
  tempAlert.update(currentSensorData.temperature < TEMP_MIN || 
                   currentSensorData.temperature > TEMP_MAX);
  systemAlert.update(checkForAlerts());
}

// -----------------------------------------------------
// Log sensor data to Serial
// -----------------------------------------------------
void logSensorData() {
  logDebugf("Gas: %d | Motion: %s | Flame: %s | Temp: %.1f°C%s",
           currentSensorData.gasValue,
           currentSensorData.motionDetected ? "YES" : "NO",
           currentSensorData.flameDetected ? "YES" : "NO",
           currentSensorData.temperature,
           systemAlert.isActive() ? " [ALERT]" : "");
}

// -----------------------------------------------------
// Update Local OLED Display
// -----------------------------------------------------
void updateDisplay() {
  display.clear();

  char tempStr[8];
  dtostrf(currentSensorData.temperature, 0, 1, tempStr);

  char imgProgressStr[8];
  dtostrf(capturedImage.progress, 0, 1, imgProgressStr);

  display.noInverse();
  display.drawString(2,1,"Flame: ");
  display.drawString(15,1, currentSensorData.flameDetected ? "YES" : "NO");
  display.drawString(2,2,"Motion: ");
  display.drawString(15,2,currentSensorData.motionDetected ? "YES" : "NO");
  display.drawString(2,3,"Gas: ");
  display.drawString(15,3,currentSensorData.gasValue > GAS_THRESHOLD ? "ALERT" : "OK");
  display.drawString(2,4,"Temperature: ");
  display.drawString(15,4,tempStr);
  if(isReceivingImage()) {
    display.drawString(2,5,"Image: ");
    display.drawString(9,5,imgProgressStr);
    display.drawString(13,5,"%");
  }
  display.inverse();
  display.drawString(2,7, systemAlert.isActive() ? "ALERT!" : "Status OK");
  display.display();
}

void handleLocalAlert() {
  digitalWrite(ledPin, systemAlert.isActive() ? HIGH : LOW);
  //digitalWrite(buzzerPin, systemAlert.isActive() ? HIGH : LOW);
}

// -----------------------------------------------------
// Update ThingSpeak 
// -----------------------------------------------------
template <typename T>
void updateThingSpeakField(const AlertState& alert, Field<T>& field, const T& currentField) {
  if (!alert.isActive() && !field.frozen) {
    field.value = currentField;
  } else if (alert.isRisingEdge()) {
    field.value = currentField;
    field.frozen = true;
  }
}

void updateThingSpeakSensorData() {
  updateThingSpeakField(gasAlert, 
                        thingSpeakSensorData.gasValue, 
                        currentSensorData.gasValue);
  
  updateThingSpeakField(tempAlert, 
                        thingSpeakSensorData.temperature, 
                        currentSensorData.temperature);
  
  updateThingSpeakField(motionAlert, 
                        thingSpeakSensorData.motionDetected, 
                        currentSensorData.motionDetected);
  
  updateThingSpeakField(flameAlert, 
                        thingSpeakSensorData.flameDetected, 
                        currentSensorData.flameDetected);
}

void resetThingSpeakAfterSend() {
  thingSpeakSensorData.gasValue.frozen = false;
  thingSpeakSensorData.motionDetected.frozen = false;
  thingSpeakSensorData.flameDetected.frozen = false;
  thingSpeakSensorData.temperature.frozen = false;
}

void sendToThingSpeak() {
  static unsigned long lastSendTime = 0; // persists across calls
  unsigned long now = millis();

  if (now - lastSendTime < COOLDOWN) {
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    logDebug("Cannot update ThingSpeak - WiFi not connected");
    return;
  }

  if (!client.connect("api.thingspeak.com", 443)) {
    logDebug("Connection to ThingSpeak failed");
    return;
  }

  // Build the request URL
  String url = "/update?api_key=" + String(THINGSPEAK_API_KEY);
  url += "&field" + String(FIELD_GAS) + "=" + String(thingSpeakSensorData.gasValue.value, 2)
      + "&field" + String(FIELD_TEMPERATURE) + "=" + String(thingSpeakSensorData.temperature.value, 2)
      + "&field" + String(FIELD_MOTION) + "=" + String(thingSpeakSensorData.motionDetected.value ? 1 : 0, 2)
      + "&field" + String(FIELD_FLAME) + "=" + String(thingSpeakSensorData.flameDetected.value ? 1 : 0, 2); 
  
  // Build the GET request
  String request = String("GET ") + url + " HTTP/1.1\r\n" +
                   "Host: api.thingspeak.com\r\n" +
                   "Connection: close\r\n\r\n";

  logDebugf("Sending to ThingSpeak: %s", url.c_str());
  client.print(request);

  // Optionally read response
  String response = client.readString();
  logDebugf("ThingSpeak response: %s", response.c_str());
  client.stop();

  lastSendTime = now;
  resetThingSpeakAfterSend();
}


// -----------------------------------------------------
// Camera State Machine
// -----------------------------------------------------
void manageCameraCapture() {
  switch(cameraState) {
    case CAM_IDLE:
      if (shouldRequestCapture()) {
        telegramCommandTakePicture = false; 
        sendCommandToCamera(CAPTURE_CMD);
        cameraState = CAM_WAITING_FOR_IMAGE;
        logDebug("State: WAITING_FOR_IMAGE");
      }
      break;
      
    case CAM_WAITING_FOR_IMAGE:
      if (tryReceiveImageData()) {
        cameraState = CAM_IMAGE_READY;
        logDebug("State: IMAGE_READY");
      }
      break;
      
    case CAM_IMAGE_READY:
      // Image will be sent by Telegram handler
      break;
  }
}

// -----------------------------------------------------
// Check if capture should be requested
// -----------------------------------------------------
bool shouldRequestCapture() {
  return flameAlert.isRisingEdge() || 
         motionAlert.isRisingEdge() || 
         telegramCommandTakePicture;
}

// -----------------------------------------------------
// Send capture command to camera board
// -----------------------------------------------------
void sendCommandToCamera(char command) {
  CameraSerial.write(command);
  logDebugf("Command sent to camera: %c", command);
}

// -----------------------------------------------------
// Try to receive image data (non-blocking)
// -----------------------------------------------------
bool tryReceiveImageData() {
  static enum {WAIT_START, READ_SIZE, READ_DATA, WAIT_END} receiveState = WAIT_START;
  static uint32_t expectedSize = 0;
  static uint32_t lastByteTime = 0;
  const size_t CHUNK_SIZE = 1024;
  const unsigned long CHUNK_TIMEOUT_MS = 15000;

  while (CameraSerial.available() > 0) {
    char incomingByte = CameraSerial.read();
    lastByteTime = millis();
    
    switch(receiveState) {
      case WAIT_START:
        if (incomingByte == RESPONSE_START) {
          logDebug("Received START marker");
          receiveState = READ_SIZE;
          expectedSize = 0;
        } else if (incomingByte == RESPONSE_ERROR) {
          logDebug("Camera reported error");
          receiveState = WAIT_START;
          return false;
        }
        break;
        
      case READ_SIZE:
        {
          static uint8_t sizeBytes[4];
          static uint8_t sizeBytesRead = 0;
          
          sizeBytes[sizeBytesRead++] = incomingByte;
          
          if (sizeBytesRead == 4) {
            expectedSize = (uint32_t)sizeBytes[0] |
                          ((uint32_t)sizeBytes[1] << 8) |
                          ((uint32_t)sizeBytes[2] << 16) |
                          ((uint32_t)sizeBytes[3] << 24);
            
            logDebugf("Expected image size: %d bytes", expectedSize);
            
            capturedImage.buffer = (uint8_t*)malloc(expectedSize);
            if (capturedImage.buffer == NULL) {
              logDebug("Failed to allocate image buffer!");
              receiveState = WAIT_START;
              return false;
            }
            
            capturedImage.size = expectedSize;
            capturedImage.bytesRead = 0;
            receiveState = READ_DATA;
            sizeBytesRead = 0;
          }
        }
        break;
        
      case READ_DATA: 
        capturedImage.buffer[capturedImage.bytesRead++] = incomingByte;
        capturedImage.progress = (float)capturedImage.bytesRead / capturedImage.size * 100.0;

        if (capturedImage.bytesRead % CHUNK_SIZE == 0 || capturedImage.bytesRead == capturedImage.size) {
            sendCommandToCamera(RESPONSE_ACK);
        }

        if (capturedImage.bytesRead >= capturedImage.size) {
          logDebugf("Received all %d bytes", capturedImage.bytesRead);
          receiveState = WAIT_END;
        }
        break;
        
      case WAIT_END:
        if (incomingByte == RESPONSE_END) {
          logDebug("Received END marker - Image complete!");
          capturedImage.available = true;
          receiveState = WAIT_START;
          return true;
        }
        break;
    }
  }
  
  // Check timeout
  if ((receiveState != WAIT_START) && (millis() - lastByteTime > CHUNK_TIMEOUT_MS)) {
    logDebug("❌ Timeout waiting for image data");
    receiveState = WAIT_START;
    if (capturedImage.buffer) {
      free(capturedImage.buffer);
      capturedImage.buffer = nullptr;
    }
    cameraState = CAM_IDLE;
    return false;
  }
  return false;
}

bool isReceivingImage() {
  return (cameraState == CAM_WAITING_FOR_IMAGE);
}

// -----------------------------------------------------
// Handle Telegram Commands
// -----------------------------------------------------
void handleTelegramCommands() {
  if (millis() - lastTelegramCheck < TELEGRAM_INTERVAL) return;
  lastTelegramCheck = millis();

  int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

  while (numNewMessages) {
    for (int i = 0; i < numNewMessages; i++) {
      String chat_id = bot.messages[i].chat_id;
      String text = bot.messages[i].text;
      text.toLowerCase();

      if (text == "/temp" || text == "/temperature") {
        telegramCommandSendTemp = true;
      } 
      else if (text == "/pic" || text == "/picture" || text == "/image") {
        telegramCommandTakePicture = true;
      }
      else if (text == "/help") {
        String help = "Available commands:\n";
        help += "/temp or /temperature - Get current temperature\n";
        help += "/pic or /picture or /image - Take a photo";
        bot.sendMessage(chat_id, help, "");
      } 
      else {
        bot.sendMessage(chat_id, "Unknown command. Try /help", "");
      }
    }
    numNewMessages = bot.getUpdates(bot.last_message_received + 1);
  }
}

// -----------------------------------------------------
// Telegram notification logic 
// -----------------------------------------------------
void handleTelegramNotifications() {
  checkAndSendAlertMessages();
  sendSensorDataIfRequested();
  sendCapturedImageIfReady();
}

void checkAndSendAlertMessages() {
  String message = "";

  if (gasAlert.isRisingEdge()) {
    message += "⚠️ Gas alert detected!\n";
  }
  if (motionAlert.isRisingEdge()) {
    message += "⚠️ Motion detected!\n";
  }
  if (flameAlert.isRisingEdge()) {
    message += "🔥 Flame detected!\n";
  }
  if (tempAlert.isRisingEdge()) {
    message += "🌡️ Temperature out of range!\n";
  }

  if (message.length() > 0) {
    bot.sendMessage(TELEGRAM_CHAT_ID, message, "");
  }
}

void sendSensorDataIfRequested() {
  if (telegramCommandSendTemp) {
    String message = "🌡️ Current Temperature: ";
    char tempStr[8];
    dtostrf(currentSensorData.temperature, 0, 1, tempStr);
    message += String(tempStr) + "°C\n";
    bot.sendMessage(TELEGRAM_CHAT_ID, message, "");
    telegramCommandSendTemp = false;
  }
}

void sendCapturedImageIfReady() {
  if (capturedImage.available && cameraState == CAM_IMAGE_READY) {
    sendPhotoToTelegram(capturedImage.buffer, capturedImage.size);
    freeImageBuffer(capturedImage);
    cameraState = CAM_IDLE;
  }
}

// -----------------------------------------------------
// Telegram Photo logic
// -----------------------------------------------------
void sendPhotoToTelegram(uint8_t* buffer, size_t length) {
  if (!buffer || length == 0) {
    logDebug("No image data to send!");
    return;
  }

  if (!client.connect(telegramHost, 443)) {
    logDebug("Connection to Telegram failed!");
    return;
  }

  String boundary = "ESP32CAMBOUNDARY";

  String head = "--" + boundary + "\r\n"
                "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n" +
                TELEGRAM_CHAT_ID +
                "\r\n--" + boundary + "\r\n"
                "Content-Disposition: form-data; name=\"photo\"; filename=\"photo.jpg\"\r\n"
                "Content-Type: image/jpeg\r\n\r\n";

  String tail = "\r\n--" + boundary + "--\r\n";

  uint32_t totalLen = head.length() + length + tail.length();

  client.printf("POST /bot%s/sendPhoto HTTP/1.1\r\n", TELEGRAM_TOKEN);
  client.printf("Host: %s\r\n", telegramHost);
  client.println("Content-Type: multipart/form-data; boundary=" + boundary);
  client.printf("Content-Length: %d\r\n\r\n", totalLen);

  client.print(head);
  client.write(buffer, length);
  client.print(tail);

  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break;
  }

  String response = client.readString();
  logDebug("[MAIN] Telegram response:");
  Serial.println(response);

  client.stop();
}

// -----------------------------------------------------
// Free the captured image buffer and reset the ImageData struct
// -----------------------------------------------------
void freeImageBuffer(ImageData &image) {
  if (image.buffer != nullptr) {
    free(image.buffer);
    image.buffer = nullptr;
  }
  image.size = 0;
  image.bytesRead = 0;
  image.progress = 0.0f;
  image.available = false;
}

// -----------------------------------------------------
// Logging Functions
// -----------------------------------------------------
void logDebug(const char* message) {
  if (!DEBUG) return;
  Serial.print("[MAIN] ");
  Serial.println(message);
}

void logDebugf(const char* format, ...) {
  if (!DEBUG) return;
  
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  Serial.print("[MAIN] ");
  Serial.println(buffer);
}