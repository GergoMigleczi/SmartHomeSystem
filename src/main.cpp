/**
 * @file        main.ccp
 * @brief       ESP32 Safety System with Telegram, ThingSpeak, and Camera Integration.
 * @details     Monitors gas, flame, motion, and temperature sensors, displays status on OLED, 
 *              and sends alerts via Telegram and ThingSpeak. Captures images on alerts or user command.
 * 
 * @author      Gergo Migleczi
 * @date        2025-10-30
 * @version     v1.0
 * 
 * @hardware    ESP32 Dev Module
 * @framework   Arduino
 * 
 * @dependencies
 *   - WiFi.h
 *   - UniversalTelegramBot.h
 *   - ThingSpeak.h
 *   - DallasTemperature.h
 *   - OLED library
 * 
 */

// =====================================================
// ==================== Includes =======================
// =====================================================

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

// =====================================================
// ==================== Debugging ======================
// =====================================================

const bool DEBUG = true;

// =====================================================
// ==================== OLED Config ====================
// =====================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

OLED display = OLED(
  21,                  // SDA pin for I2C
  22,                  // SCL pin for I2C
  NO_RESET_PIN,        // Reset pin (none)
  OLED::W_128,         // Display width
  OLED::H_64,          // Display height
  OLED::CTRL_SH1106,   // Display controller chip
  0x3C                 // I2C address
);

// =====================================================
// ==================== Pin Config =====================
// =====================================================

const int pirPin = 2;
const int flamePin = 4;
const int ledPin = 5;
const int tempPin = 15;
const int CameraPin_RX = 16;
const int CameraPin_TX = 17;
const int buzzerPin = 18;
const int gasPin = 34;

// =====================================================
// ==================== Thresholds =====================
// =====================================================

const float TEMP_MIN = 6.0;
const float TEMP_MAX = 30.0;
const int GAS_THRESHOLD = 3000;

// =====================================================
// ==================== Wi-Fi Setup ====================
// =====================================================

WiFiClientSecure client;

// =====================================================
// ==================== Telegram =======================
// =====================================================

const char* telegramHost = "api.telegram.org";
UniversalTelegramBot bot(TELEGRAM_TOKEN, client);

// Telegram polling interval
unsigned long lastTelegramCheck = 0;
const unsigned long TELEGRAM_INTERVAL = 2000;

bool telegramCommandTakePicture = false;
bool telegramCommandSendTemp = false;
bool telegramCommandSendHelp = false;

// =====================================================
// ================== ThingSpeak =======================
// =====================================================

const unsigned long COOLDOWN = 20000; // 20s delay between updates
int FIELD_GAS = 1;
int FIELD_TEMPERATURE = 2;
int FIELD_FLAME = 3;
int FIELD_MOTION = 4;

// =====================================================
// ================= Temperature =======================
// =====================================================

OneWire oneWire(tempPin);
DallasTemperature tempSensor(&oneWire);

// =====================================================
// ================== Camera UART ======================
// =====================================================

HardwareSerial CameraSerial(2);
const int UART_BAUD_RATE = 115200;
const char CAPTURE_CMD = 'C';
const char RESPONSE_START = 'S';
const char RESPONSE_END = 'E';
const char RESPONSE_ERROR = 'X';
const char RESPONSE_ACK = 'A';
const char BLINK_CMD = 'B';

// =====================================================
// ================== Camera State =====================
// =====================================================

enum CameraState {
  CAM_IDLE,
  CAM_WAITING_FOR_IMAGE,
  CAM_IMAGE_READY
};

CameraState cameraState = CAM_IDLE;

// =====================================================
// ================= Data Structures ===================
// =====================================================

/**
 * @brief Stores captured image data and transfer progress.
 */
struct ImageData {
  uint8_t* buffer;
  uint32_t size;
  uint32_t bytesRead;
  float progress;
  bool available;
};

ImageData capturedImage = {NULL, 0, 0, 0.0, false};

/**
 * @brief Stores current sensor readings.
 */
struct SensorData {
  int gasValue;
  bool motionDetected;
  bool flameDetected;
  float temperature;
};

SensorData currentSensorData {0, false, false, 0.0};

/**
 * @brief Template wrapper for ThingSpeak fields (supports freezing on alerts).
 */
template <typename T>
struct Field {
  T value;
  bool frozen = false;
};

/**
 * @brief Stores ThingSpeak-ready sensor values.
 */
struct ThingSpeakSensorData {
  Field<int> gasValue;
  Field<bool> motionDetected;
  Field<bool> flameDetected;
  Field<float> temperature;
};

ThingSpeakSensorData thingSpeakSensorData;

/**
 * @brief Tracks alert states with rising edge detection.
 */
struct AlertState {
  bool current;
  bool previous;

  void update(bool condition) {
    previous = current;
    current = condition;
  }

  bool isRisingEdge() const { return current && !previous; }
  bool isActive() const { return current; }
};

// =====================================================
// ================= Alert States ======================
// =====================================================

AlertState gasAlert;
AlertState motionAlert;
AlertState flameAlert;
AlertState tempAlert;
AlertState systemAlert;

// =====================================================
// =============== Function Declarations ===============
// =====================================================

void initializeSystem();
SensorData readSensors();
bool checkForAlerts();
void updateAllAlertStates();
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

// =====================================================
// ====================== Setup ========================
// =====================================================

/**
 * @brief  Initialises serial communication and system components.
 */
void setup() {
  Serial.begin(115200);
  initializeSystem();
}

// =====================================================
// ======================= Loop ========================
// =====================================================

/**
 * @brief  Main control loop for reading sensors, handling alerts, 
 *         managing Telegram communication, and ThingSpeak updates.
 */
void loop() {
  currentSensorData = readSensors();
  updateAllAlertStates();  
  updateDisplay();
  handleLocalAlert();
  
  handleTelegramCommands();
  manageCameraCapture();  
  handleTelegramNotifications();

  updateThingSpeakSensorData();
  sendToThingSpeak();
  
  delay(500);
}

// =====================================================
// ============== System Initialisation ================
// =====================================================

/**
 * @brief  Sets up GPIOs, peripherals, and network connections.
 */
void initializeSystem() {
  pinMode(pirPin, INPUT);
  pinMode(flamePin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  tempSensor.begin();

  // --- Camera setup ---
  CameraSerial.setRxBufferSize(1024);
  CameraSerial.setTxBufferSize(1024);
  CameraSerial.begin(UART_BAUD_RATE, SERIAL_8N1, CameraPin_RX, CameraPin_TX);
  logDebug("Camera UART initialised");
  sendCommandToCamera(BLINK_CMD);

  // --- OLED setup ---
  display.begin();
  display.clear();
  display.drawString(2, 1, "System Initialising...");
  display.display();

  // --- Wi-Fi connection ---
  logDebug("Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  display.drawString(2, 2, "Connecting to WiFi...");
  display.display();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  logDebug("Wi-Fi connected!");
  client.setInsecure();
  bot.sendMessage(TELEGRAM_CHAT_ID, "ESP32 Safety System online!", "");

  // --- ThingSpeak setup ---
  ThingSpeak.begin(client);
  logDebug("ThingSpeak initialised");

  delay(2000);
}

// =====================================================
// ================ Sensor Reading =====================
// =====================================================

/**
 * @brief  Reads sensor data from gas, flame, motion, and temperature sensors.
 * @return Struct containing the latest sensor readings.
 */
SensorData readSensors() {
  SensorData data;
  data.gasValue = analogRead(gasPin);
  data.motionDetected = digitalRead(pirPin);
  data.flameDetected = (digitalRead(flamePin) == 1 ? false : true);
  tempSensor.requestTemperatures();
  data.temperature = tempSensor.getTempCByIndex(0);
  return data;
}

// =====================================================
// ================ Alert Management ===================
// =====================================================

/**
 * @brief  Checks if any sensor alert condition is active.
 * @return True if at least one alert is active.
 */
bool checkForAlerts() {
  return gasAlert.isActive() || motionAlert.isActive() ||
         flameAlert.isActive() || tempAlert.isActive();
}

/**
 * @brief  Updates all alert states based on current sensor readings.
 */
void updateAllAlertStates() {
  gasAlert.update(currentSensorData.gasValue > GAS_THRESHOLD);
  motionAlert.update(currentSensorData.motionDetected);
  flameAlert.update(currentSensorData.flameDetected);
  tempAlert.update(currentSensorData.temperature < TEMP_MIN ||
                   currentSensorData.temperature > TEMP_MAX);
  systemAlert.update(checkForAlerts());
}

/**
 * @brief  Controls LED and buzzer based on alert state.
 */
void handleLocalAlert() {
  digitalWrite(ledPin, systemAlert.isActive() ? HIGH : LOW);
  digitalWrite(buzzerPin, systemAlert.isActive() ? HIGH : LOW);
}

// =====================================================
// ==================== Display ========================
// =====================================================

/**
 * @brief  Updates OLED screen with sensor and system status.
 */
void updateDisplay() {
  display.clear();
  display.noInverse();

  char tempStr[8];
  dtostrf(currentSensorData.temperature, 0, 1, tempStr);

  char imgProgressStr[8];
  dtostrf(capturedImage.progress, 0, 1, imgProgressStr);

  display.drawString(2, 1, "Flame: ");
  display.drawString(15, 1, currentSensorData.flameDetected ? "YES" : "NO");
  display.drawString(2, 2, "Motion: ");
  display.drawString(15, 2, currentSensorData.motionDetected ? "YES" : "NO");
  display.drawString(2, 3, "Gas: ");
  display.drawString(15, 3, currentSensorData.gasValue > GAS_THRESHOLD ? "ALERT" : "OK");
  display.drawString(2, 4, "Temp: ");
  display.drawString(15, 4, tempStr);

  if (isReceivingImage()) {
    display.drawString(2, 5, "Image: ");
    display.drawString(9, 5, imgProgressStr);
    display.drawString(13, 5, "%");
  }

  display.inverse();
  display.drawString(2, 7, systemAlert.isActive() ? "ALERT!" : "Status OK");
  display.display();
}

// =====================================================
// ================ ThingSpeak Logic ===================
// =====================================================

/**
 * @brief  Updates ThingSpeak field data based on current readings and alert state.
 */
template <typename T>
void updateThingSpeakField(const AlertState& alert, Field<T>& field, const T& currentField) {
  if(alert.isActive()){
    field.value = currentField;
    field.frozen = true;
  } else if (!field.frozen){
    field.value = currentField;
  }
}

/**
 * @brief  Updates all ThingSpeak fields using current sensor data and alert logic.
 */
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

/**
 * @brief  Sends current readings to ThingSpeak if cooldown has elapsed.
 */
void sendToThingSpeak() {
  static unsigned long lastSendTime = 0;
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

  // --- Build the request URL ---
  String url = "/update?api_key=" + String(THINGSPEAK_API_KEY);
  url += "&field" + String(FIELD_GAS) + "=" + String(thingSpeakSensorData.gasValue.value, 2)
      + "&field" + String(FIELD_TEMPERATURE) + "=" + String(thingSpeakSensorData.temperature.value, 2)
      + "&field" + String(FIELD_MOTION) + "=" + String(thingSpeakSensorData.motionDetected.value ? 1 : 0, 2)
      + "&field" + String(FIELD_FLAME) + "=" + String(thingSpeakSensorData.flameDetected.value ? 1 : 0, 2); 
  
  // --- Build the GET request ---
  String request = String("GET ") + url + " HTTP/1.1\r\n" +
                   "Host: api.thingspeak.com\r\n" +
                   "Connection: close\r\n\r\n";

  logDebugf("Sending to ThingSpeak: %s", url.c_str());
  client.print(request);

  // --- Read response ---
  String response = client.readString();
  logDebugf("ThingSpeak response: %s", response.c_str());
  client.stop();

  lastSendTime = now;

  // --- Reset frozen states after successful send ---
  resetThingSpeakAfterSend();
}



// =====================================================
// =================== Camera Logic ====================
// =====================================================

/**
 * @brief  Determines if a camera capture should be triggered (alert or Telegram command).
 * @return True if capture should be requested.
 */
bool shouldRequestCapture() {
  return flameAlert.isRisingEdge() || 
         motionAlert.isRisingEdge() || 
         telegramCommandTakePicture;
}

/**
 * @brief  Sends a command to the camera module over UART.
 * @param  command Character representing the command ('C' = capture, 'B' = blink).
 */
void sendCommandToCamera(char command) {
  CameraSerial.write(command);
  logDebugf("Command sent to camera: %c", command);
}

/**
 * @brief  Checks if image data is currently being received from camera.
 * @return True if image reception is in progress.
 */
bool isReceivingImage() {
  return (cameraState == CAM_WAITING_FOR_IMAGE);
}

/**
 * @brief  Attempts to read image data from the camera if available.
 * @details Implements a simple state machine that waits for the start marker,
 *          reads the image size, receives data in chunks, and verifies the end marker.
 *          Handles timeouts and memory allocation failures gracefully.
 * @return  True if image fully received; false otherwise.
 */
bool tryReceiveImageData() {
  // --- Persistent state machine variables ---
  static enum { WAIT_START, READ_SIZE, READ_DATA, WAIT_END } receiveState = WAIT_START;
  static uint32_t expectedSize = 0;
  static uint32_t lastByteTime = 0;

  // --- Constants for chunking and timeout handling ---
  const size_t CHUNK_SIZE = 1024;
  const unsigned long CHUNK_TIMEOUT_MS = 15000;

  // --- Process all available bytes in serial buffer ---
  while (CameraSerial.available() > 0) {
    char incomingByte = CameraSerial.read();
    lastByteTime = millis(); // reset timeout timer on each received byte
    
    switch (receiveState) {
      // ==========================================================
      // WAIT_START: Wait for start marker or error indicator
      // ==========================================================
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

      // ==========================================================
      // READ_SIZE: Read 4 bytes representing expected image size
      // ==========================================================
      case READ_SIZE: {
        static uint8_t sizeBytes[4];
        static uint8_t sizeBytesRead = 0;
        
        sizeBytes[sizeBytesRead++] = incomingByte;

        if (sizeBytesRead == 4) {
          // --- Combine bytes into a 32-bit image size ---
          expectedSize = (uint32_t)sizeBytes[0] |
                        ((uint32_t)sizeBytes[1] << 8) |
                        ((uint32_t)sizeBytes[2] << 16) |
                        ((uint32_t)sizeBytes[3] << 24);

          logDebugf("Expected image size: %d bytes", expectedSize);

          // --- Allocate buffer to hold incoming image data ---
          capturedImage.buffer = (uint8_t*)malloc(expectedSize);
          if (capturedImage.buffer == NULL) {
            logDebug("Failed to allocate image buffer!");
            receiveState = WAIT_START;
            return false;
          }

          // --- Initialise progress tracking ---
          capturedImage.size = expectedSize;
          capturedImage.bytesRead = 0;
          receiveState = READ_DATA;
          sizeBytesRead = 0; // reset for next transmission
        }
      } break;

      // ==========================================================
      // READ_DATA: Store image bytes into buffer and track progress
      // ==========================================================
      case READ_DATA:
        capturedImage.buffer[capturedImage.bytesRead++] = incomingByte;
        capturedImage.progress = (float)capturedImage.bytesRead / capturedImage.size * 100.0;

        // --- Acknowledge receipt of every chunk to maintain sync ---
        if (capturedImage.bytesRead % CHUNK_SIZE == 0 || capturedImage.bytesRead == capturedImage.size) {
          sendCommandToCamera(RESPONSE_ACK);
        }

        // --- Once all expected bytes are received, wait for END marker ---
        if (capturedImage.bytesRead >= capturedImage.size) {
          logDebugf("Received all %d bytes", capturedImage.bytesRead);
          receiveState = WAIT_END;
        }
        break;

      // ==========================================================
      // WAIT_END: Confirm transmission completion via end marker
      // ==========================================================
      case WAIT_END:
        if (incomingByte == RESPONSE_END) {
          logDebug("Received END marker - Image complete!");
          capturedImage.available = true;
          receiveState = WAIT_START; // reset for next image
          return true;
        }
        break;
    }
  }

  // ==========================================================
  // Timeout Handling
  // ==========================================================
  if ((receiveState != WAIT_START) && (millis() - lastByteTime > CHUNK_TIMEOUT_MS)) {
    logDebug("❌ Timeout waiting for image data");
    receiveState = WAIT_START;

    // --- Clean up partial buffer if timeout occurred ---
    if (capturedImage.buffer) {
      free(capturedImage.buffer);
      capturedImage.buffer = nullptr;
    }

    cameraState = CAM_IDLE; // return to idle mode
    return false;
  }

  return false;
}

/**
 * @brief  Manages camera operations (trigger, receive, and store images).
 */
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
      // --- Image will be sent by Telegram handler ---
      break;
  }
}

/**
 * @brief  Frees memory allocated for an image buffer.
 * @param  image ImageData structure to clear.
 */
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

// =====================================================
// ================= Telegram Bot ======================
// =====================================================

/**
 * @brief  Checks for and processes incoming Telegram commands.
 */
void handleTelegramCommands() {
  // --- Rate limit command checks to avoid spamming API ---
  if (millis() - lastTelegramCheck < TELEGRAM_INTERVAL) return;
  lastTelegramCheck = millis();

  // --- Get all new messages since last update ---
  int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

  // --- Process all pending messages ---
  while (numNewMessages) {
    for (int i = 0; i < numNewMessages; i++) {
      String chat_id = bot.messages[i].chat_id;
      String text = bot.messages[i].text;
      text.toLowerCase();  // Normalise command text

      // --- Match recognised commands ---
      if (text == "/temp" || text == "/temperature") {
        telegramCommandSendTemp = true;
      } 
      else if (text == "/pic" || text == "/picture" || text == "/image") {
        telegramCommandTakePicture = true;
      }
      else if (text == "/help") {
        telegramCommandSendHelp = true;
      } 
      else {
        // --- Handle unknown input ---
        bot.sendMessage(chat_id, "Unknown command. Try /help", "");
      }
    }

    // --- Check again in case more messages arrived mid-loop ---
    numNewMessages = bot.getUpdates(bot.last_message_received + 1);
  }
}

/**
 * @brief  Sends Telegram alerts and requested data periodically.
 */
void handleTelegramNotifications() {
  checkAndSendAlertMessages();
  sendSensorDataIfRequested();
  sendCapturedImageIfReady();
}

/**
 * @brief  Sends alert messages when new alerts are detected.
 */
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

/**
 * @brief  Sends temperature data to Telegram if requested by user.
 */
void sendSensorDataIfRequested() {
  if (telegramCommandSendTemp) {
    String message = "🌡️ Current Temperature: ";
    char tempStr[8];
    dtostrf(currentSensorData.temperature, 0, 1, tempStr);
    message += String(tempStr) + "°C\n";
    bot.sendMessage(TELEGRAM_CHAT_ID, message, "");
    telegramCommandSendTemp = false;
  }

  if (telegramCommandSendHelp) {
    String helpMessage = "🤖 Available Commands:\n"
                         "/temp or /temperature - Get current temperature\n"
                         "/pic or /picture or /image - Capture and send a picture\n"
                         "/help - Show this help message";
    bot.sendMessage(TELEGRAM_CHAT_ID, helpMessage, "");
    telegramCommandSendHelp = false;
  }
}

/**
 * @brief  Sends captured image to Telegram if image is ready.
 */
void sendCapturedImageIfReady() {
  if (capturedImage.available && cameraState == CAM_IMAGE_READY) {
    sendPhotoToTelegram(capturedImage.buffer, capturedImage.size);
    freeImageBuffer(capturedImage);
    cameraState = CAM_IDLE;
  }
}

/**
 * @brief  Sends a binary image buffer to Telegram as a photo.
 * @param  buffer Pointer to the image byte array.
 * @param  length Length of image data.
 */
void sendPhotoToTelegram(uint8_t* buffer, size_t length) {
  // --- Validate input ---
  if (!buffer || length == 0) {
    logDebug("No image data to send!");
    return;
  }

  // --- Connect to Telegram server over HTTPS (port 443) ---
  if (!client.connect(telegramHost, 443)) {
    logDebug("Connection to Telegram failed!");
    return;
  }

  // --- Multipart boundary marker (used to separate form fields) ---
  String boundary = "ESP32CAMBOUNDARY";

  // --- Build HTTP form-data headers (chat_id + image file) ---
  String head = "--" + boundary + "\r\n"
                "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n" +
                TELEGRAM_CHAT_ID +
                "\r\n--" + boundary + "\r\n"
                "Content-Disposition: form-data; name=\"photo\"; filename=\"photo.jpg\"\r\n"
                "Content-Type: image/jpeg\r\n\r\n";

  // --- Multipart closing boundary ---
  String tail = "\r\n--" + boundary + "--\r\n";

  // --- Calculate total content length for HTTP header ---
  uint32_t totalLen = head.length() + length + tail.length();

  // --- Construct HTTP POST request ---
  client.printf("POST /bot%s/sendPhoto HTTP/1.1\r\n", TELEGRAM_TOKEN);
  client.printf("Host: %s\r\n", telegramHost);
  client.println("Content-Type: multipart/form-data; boundary=" + boundary);
  client.printf("Content-Length: %d\r\n\r\n", totalLen);

  // --- Send request body ---
  client.print(head);          // Form-data header
  client.write(buffer, length); // Image binary
  client.print(tail);          // End of multipart form

  // --- Wait for server response ---
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break; // End of headers
  }

  // --- Read and print the full response ---
  String response = client.readString();
  logDebug("[MAIN] Telegram response:");
  Serial.println(response);

  // --- Close connection ---
  client.stop();
}


// =====================================================
// ==================== Debugging ======================
// =====================================================

/**
 * @brief  Prints a simple debug message to Serial if DEBUG mode is enabled.
 * @param  message Message to print.
 */
void logDebug(const char* message) {
  if (!DEBUG) return;
  Serial.print("[MAIN] ");
  Serial.println(message);
}

/**
 * @brief  Prints a formatted debug message to Serial (printf-style).
 * @param  format Printf-style format string.
 * @param  ... Arguments to format.
 */
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

// =====================================================
// ===================== End of File ===================
// =====================================================