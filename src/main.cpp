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
const char RESPONSE_START = 'S'; // Start of image transmission
const char RESPONSE_END = 'E';   // End of image transmission
const char RESPONSE_ERROR = 'X'; // Error during capture

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
  bool available;       // Is image ready to send?
};

ImageData capturedImage = {NULL, 0, 0, false};

// ==== Sensor Data Structure ====
struct SensorData {
  int gasValue;
  bool motionDetected;
  bool flameDetected;
  float temperature;
};

// ==== Alert State Tracking ====
bool gasAlertState = false;
bool motionAlertState = false;
bool flameAlertState = false;
bool tempAlertState = false;

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
void sendCaptureCommand();
bool tryReceiveImageData();
void freeImageBuffer();

void handleTelegramNotifications();
void sendImageToTelegram();

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
  SensorData data = readSensors();
  bool alert = checkForAlerts(data);

  updateAlertStates(data);  // Update global alert states once
  
  logSensorData(data, alert);
  updateDisplay(data, alert);
  handleLocalAlert(alert);
  
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
  CameraSerial.begin(115200, SERIAL_8N1, CameraPin_RX, CameraPin_TX);
  logStatus("Camera UART initialized");

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    logStatus("SSD1306 allocation failed");
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("System Initializing..."));
  display.display();

  // Wi-Fi connection
  logStatus("Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
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

void handleLocalAlert(bool alert) {
  digitalWrite(ledPin, alert ? HIGH : LOW);
  digitalWrite(buzzerPin, alert ? HIGH : LOW);
}

// -----------------------------------------------------
// Camera State Machine
// -----------------------------------------------------
void manageCameraCapture() {
  switch(cameraState) {
    case CAM_IDLE:
      // Check if we should request a capture
      if (shouldRequestCapture()) {
        sendCaptureCommand();
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
void sendCaptureCommand() {
  CameraSerial.write(CAPTURE_CMD);
  logStatus("📷 Capture command sent to camera");
}

// -----------------------------------------------------
// Try to receive image data (non-blocking)
// -----------------------------------------------------
bool tryReceiveImageData() {
  static enum {WAIT_START, READ_SIZE, READ_DATA, WAIT_END} receiveState = WAIT_START;
  static uint32_t expectedSize = 0;
  
  while (CameraSerial.available() > 0) {
    char incomingByte = CameraSerial.read();
    
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
  
  return false;
}

// -----------------------------------------------------
// Free image buffer memory
// -----------------------------------------------------
void freeImageBuffer() {
  if (capturedImage.buffer != NULL) {
    free(capturedImage.buffer);
    capturedImage.buffer = NULL;
  }
  capturedImage.size = 0;
  capturedImage.bytesRead = 0;
  capturedImage.available = false;
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
    sendImageToTelegram();
  }
}

// -----------------------------------------------------
// Telegram Image Upload Callbacks
// -----------------------------------------------------
static uint32_t imageUploadIndex = 0;

// Check if more data is available
bool imageMoreDataAvailable() {
  return imageUploadIndex < capturedImage.size;
}

// Get next byte
uint8_t imageGetNextByte() {
  return capturedImage.buffer[imageUploadIndex++];
}

// Get pointer to next buffer chunk
uint8_t* imageGetNextBuffer() {
  return capturedImage.buffer + imageUploadIndex;
}

// Get length of next buffer chunk
int imageGetNextBufferLen() {
  const int CHUNK_SIZE = 1024;
  int remaining = capturedImage.size - imageUploadIndex;
  int chunkSize = min(CHUNK_SIZE, remaining);
  imageUploadIndex += chunkSize;
  return chunkSize;
}

// -----------------------------------------------------
// Send image to Telegram
// -----------------------------------------------------
void sendImageToTelegram() {
  logStatusf("Sending image to Telegram (%d bytes)", capturedImage.size);
  
  // Reset upload index
  imageUploadIndex = 0;
  
  // Send photo to Telegram using the correct API
  String response = bot.sendMultipartFormDataToTelegram(
    "sendPhoto",
    "photo",
    "image.jpg",
    "image/jpeg",
    TELEGRAM_CHAT_ID,
    capturedImage.size,
    imageMoreDataAvailable,
    imageGetNextByte,
    imageGetNextBuffer,
    imageGetNextBufferLen
  );
  
  if (response.indexOf("\"ok\":true") > 0) {
    logStatus("✅ Image sent successfully!");
  } else {
    logStatus("❌ Failed to send image");
    logStatusf("Response: %s", response.c_str());
  }
  
  // Clean up and reset state
  freeImageBuffer();
  cameraState = CAM_IDLE;
  logStatus("State: IDLE");
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