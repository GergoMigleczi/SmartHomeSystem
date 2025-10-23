#include <Arduino.h>
#include <Wire.h>
#include <oled.h>
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
OLED display = OLED (21,                       // sda pin for I2C comunication
         22,                       // scl pin for I2C comunication
         NO_RESET_PIN,        // Reset pin (default: none)
         OLED::W_128, // Display width, must be one of enum: W_96 or W_128 (default: W_128).
         OLED::H_64, // Display height, must be one of enum: H_16, H_32 or OLED::H_64 (default: H_32).
         OLED::CTRL_SH1106,     //Display controller chip, must be one of enum: CTRL_SH1106 or CTRL_SSD1306 (default: CTRL_SSD1306).                  
         0x3C               // I2C address (default: 0x3C)
    );

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
const char* telegramHost = "api.telegram.org"; // Telegram API server host
UniversalTelegramBot bot(TELEGRAM_TOKEN, client);

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

// ==== Alert State Tracking ====
bool gasAlertState = false;
bool motionAlertState = false;
bool flameAlertState = false;
bool tempAlertState = false;
bool systemAlertState = false;

// Previous states for edge detection
bool prevGasAlertState = false;
bool prevMotionAlertState = false;
bool prevFlameAlertState = false;
bool prevTempAlertState = false;

// -----------------------------------------------------
// Function declarations
// -----------------------------------------------------
void initializeSystem();
SensorData readSensors();
bool checkForAlerts(const SensorData &data);
void updateAlertStates(const SensorData &data);
void logSensorData(const SensorData &data, bool alert);
void updateDisplay(const SensorData &data, bool alert);
void handleLocalAlert(bool alert);

void manageCameraCapture();
bool shouldRequestCapture();
void sendCommandToCamera(char command);
bool tryReceiveImageData();

void handleTelegramNotifications();
void sendPhotoToTelegram(uint8_t* buffer, size_t length);

void freeImageBuffer(ImageData &image);

void logStatus(const char* message);
void logStatusf(const char* format, ...);

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
  currentSensorData = readSensors();
  systemAlertState = checkForAlerts(currentSensorData);

  updateAlertStates(currentSensorData);  // Update global alert states once
  
  //logSensorData(currentSensorData, systemAlertState);
  updateDisplay(currentSensorData, systemAlertState);
  handleLocalAlert(systemAlertState);
  
  manageCameraCapture();  // Handle all camera logic via state machine
  
  handleTelegramNotifications();

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
  logStatus("Camera UART initialized");
  sendCommandToCamera(BLINK_CMD); // Blink camera LED to indicate ready

  // Initialize OLED
  display.begin();
  display.clear();
  display.noInverse();
  display.drawString(2,1,"System Initialising...");
  display.display();
  

  // Wi-Fi connection
  logStatus("Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  display.drawString(2,2,"Connecting to Wifi...");
  display.display();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  logStatus("Connected!");

  client.setInsecure(); // skip certificate check for Telegram
  bot.sendMessage(TELEGRAM_CHAT_ID, "ESP32 Safety System is online!", "");

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

bool checkForAlerts(const SensorData &data) {
  bool gasAlert = data.gasValue > GAS_THRESHOLD;
  bool tempAlert = (data.temperature < TEMP_MIN || data.temperature > TEMP_MAX);
  return gasAlert || data.motionDetected || data.flameDetected || tempAlert;
}

// -----------------------------------------------------
// Update global alert states from sensor data
// -----------------------------------------------------
void updateAlertStates(const SensorData &data) {
  // Store previous states for edge detection
  prevGasAlertState = gasAlertState;
  prevMotionAlertState = motionAlertState;
  prevFlameAlertState = flameAlertState;
  prevTempAlertState = tempAlertState;
  
  // Update current states
  gasAlertState = data.gasValue > GAS_THRESHOLD;
  motionAlertState = data.motionDetected;
  flameAlertState = data.flameDetected;
  tempAlertState = (data.temperature < TEMP_MIN || data.temperature > TEMP_MAX);
}

// -----------------------------------------------------
// Display and Output
// -----------------------------------------------------
void logSensorData(const SensorData &data, bool alert) {
  logStatusf("Gas: %d | Motion: %s | Flame: %s | Temp: %.1f°C%s",
           data.gasValue,
           data.motionDetected ? "YES" : "NO",
           data.flameDetected ? "YES" : "NO",
           data.temperature,
           alert ? " [ALERT]" : "");
}

void updateDisplay(const SensorData &data, bool alert) {
  display.clear();

  char tempStr[8];
  dtostrf(data.temperature, 0, 1, tempStr);

  char imgProgressStr[8];
  dtostrf(capturedImage.progress, 0, 1, imgProgressStr);

  display.noInverse();
  display.drawString(2,1,"Flame: ");
  display.drawString(15,1, data.flameDetected ? "YES" : "NO");
  display.drawString(2,2,"Motion: ");
  display.drawString(15,2,data.motionDetected ? "YES" : "NO");
  display.drawString(2,3,"Gas: ");
  display.drawString(15,3,data.gasValue > GAS_THRESHOLD ? "ALERT" : "OK");
  display.drawString(2,4,"Temperature: ");
  display.drawString(15,4,tempStr);
  if(cameraState == CAM_WAITING_FOR_IMAGE) {
    display.drawString(2,5,"Image: ");
    display.drawString(9,5,imgProgressStr);
    display.drawString(13,5,"%");
  }
  display.inverse();
  display.drawString(2,7, alert ? "ALERT!" : "Status OK"); // first line
  display.display();
}

void handleLocalAlert(bool alert) {
  digitalWrite(ledPin, alert ? HIGH : LOW);
  //digitalWrite(buzzerPin, alert ? HIGH : LOW);
}

// -----------------------------------------------------
// Camera State Machine
// -----------------------------------------------------
void manageCameraCapture() {
  switch(cameraState) {
    case CAM_IDLE:
      // Check if we should request a capture
      if (shouldRequestCapture()) {
        sendCommandToCamera(CAPTURE_CMD);
        //cameraState = CAM_WAITING_FOR_IMAGE;
        cameraState = CAM_WAITING_FOR_IMAGE;
        logStatus("State: WAITING_FOR_IMAGE");
      }
      break;
      
    case CAM_WAITING_FOR_IMAGE:
      // Try to receive image data (non-blocking)
      if (tryReceiveImageData()) {
        cameraState = CAM_IMAGE_READY;
        logStatus("State: IMAGE_READY");
      }
      break;
      
    case CAM_IMAGE_READY:
      // Image will be sent by Telegram handler
      // State will be reset to IDLE after sending
      break;
  }
}

// -----------------------------------------------------
// Check if capture should be requested
// -----------------------------------------------------
bool shouldRequestCapture() {
  // Request capture on rising edge (alert just turned on)
  bool flameTriggered = flameAlertState && !prevFlameAlertState;
  bool motionTriggered = motionAlertState && !prevMotionAlertState;
  
  return flameTriggered || motionTriggered;
}

// -----------------------------------------------------
// Send capture command to camera board
// -----------------------------------------------------
void sendCommandToCamera(char command) {
  CameraSerial.write(command);
  logStatusf("Command sent to camera: %c", command);
}

// -----------------------------------------------------
// Try to receive image data (non-blocking)
// -----------------------------------------------------
bool tryReceiveImageData() {
  static enum {WAIT_START, READ_SIZE, READ_DATA, WAIT_END} receiveState = WAIT_START;
  static uint32_t expectedSize = 0;
  static uint32_t lastByteTime = 0; // last time a byte was received
  const size_t CHUNK_SIZE = 1024;
  const unsigned long  CHUNK_TIMEOUT_MS = 15000; // 15 seconds timeout

  while (CameraSerial.available() > 0) {
    char incomingByte = CameraSerial.read();
    lastByteTime = millis(); // reset timeout counter whenever a byte arrives
    
    switch(receiveState) {
      case WAIT_START:
        if (incomingByte == RESPONSE_START) {
          logStatus("Received START marker");
          receiveState = READ_SIZE;
          expectedSize = 0;
        } else if (incomingByte == RESPONSE_ERROR) {
          logStatus("Camera reported error");
          receiveState = WAIT_START;
          return false;
        }
        break;
        
      case READ_SIZE:
        // Read 4 bytes for size (little-endian)
        static uint8_t sizeBytes[4];
        static uint8_t sizeBytesRead = 0;
        
        sizeBytes[sizeBytesRead++] = incomingByte;
        
        if (sizeBytesRead == 4) {
          expectedSize = (uint32_t)sizeBytes[0] |
                        ((uint32_t)sizeBytes[1] << 8) |
                        ((uint32_t)sizeBytes[2] << 16) |
                        ((uint32_t)sizeBytes[3] << 24);
          
          logStatusf("Expected image size: %d bytes", expectedSize);
          
          // Allocate buffer for image
          capturedImage.buffer = (uint8_t*)malloc(expectedSize);
          if (capturedImage.buffer == NULL) {
            logStatus("Failed to allocate image buffer!");
            receiveState = WAIT_START;
            return false;
          }
          
          capturedImage.size = expectedSize;
          capturedImage.bytesRead = 0;
          receiveState = READ_DATA;
          sizeBytesRead = 0;
        }
        break;
        
      case READ_DATA: 
        capturedImage.buffer[capturedImage.bytesRead++] = incomingByte;
        capturedImage.progress = (float)capturedImage.bytesRead / capturedImage.size * 100.0;

        // Send ACK for every CHUNK_SIZE bytes received
        if (capturedImage.bytesRead % CHUNK_SIZE == 0 || capturedImage.bytesRead == capturedImage.size) {
            sendCommandToCamera(RESPONSE_ACK);
        }
        
        //logStatusf("Image receive pending: %.1f%%, %d bytes", capturedImage.progress, capturedImage.bytesRead);

        if (capturedImage.bytesRead >= capturedImage.size) {
          logStatusf("Received all %d bytes", capturedImage.bytesRead);
          receiveState = WAIT_END;
        }
        break;
        
      case WAIT_END:
        if (incomingByte == RESPONSE_END) {
          logStatus("Received END marker - Image complete!");
          capturedImage.available = true;
          receiveState = WAIT_START;
          return true;
        }
        break;
    }
  }
  
  // Check timeout
    if ((receiveState != WAIT_START) && (millis() - lastByteTime > CHUNK_TIMEOUT_MS)) {
        logStatus("❌ Timeout waiting for image data");
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

// -----------------------------------------------------
// Telegram notification logic
// -----------------------------------------------------
void handleTelegramNotifications() {
  String message = "";

  // Check for new alerts (rising edge detection)
  if (gasAlertState && !prevGasAlertState) {
    message += "⚠️ Gas alert detected!\n";
  }
  if (motionAlertState && !prevMotionAlertState) {
    message += "⚠️ Motion detected!\n";
  }
  if (flameAlertState && !prevFlameAlertState) {
    message += "🔥 Flame detected!\n";
  }
  if (tempAlertState && !prevTempAlertState) {
    message += "🌡️ Temperature out of range!\n";
  }

  if (message.length() > 0) {
    bot.sendMessage(TELEGRAM_CHAT_ID, message, "");
  }

  // Send image if available
  if (capturedImage.available && cameraState == CAM_IMAGE_READY) {
    sendPhotoToTelegram(capturedImage.buffer, capturedImage.size);
    freeImageBuffer(capturedImage);
    cameraState = CAM_IDLE;
  }
}


void sendPhotoToTelegram(uint8_t* buffer, size_t length) {

  if (!buffer || length == 0) {
    logStatus("No image data to send!");
    return;
  }

  if (!client.connect(telegramHost, 443)) {
    logStatus("Connection to Telegram failed!");
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

  // --- Read Telegram server response (optional) ---
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break;
  }

  String response = client.readString();
  Serial.println("[MAIN] Telegram response:");
  Serial.println(response);
}

// -----------------------------------------------------
// Free the captured image buffer and reset the ImageData struct
// -----------------------------------------------------
void freeImageBuffer(ImageData &image) {
    if (image.buffer != nullptr) {
        free(image.buffer);       // Free the allocated memory
        image.buffer = nullptr;   // Avoid dangling pointer
    }
    image.size = 0;
    image.bytesRead = 0;
    image.progress = 0.0f;
    image.available = false;
}


// -----------------------------------------------------
// Logging Functions
// -----------------------------------------------------
void logStatus(const char* message) {
  Serial.print("[MAIN] ");
  Serial.println(message);
}

void logStatusf(const char* format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  Serial.print("[MAIN] ");
  Serial.println(buffer);
}