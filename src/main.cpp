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

void handleTelegramNotifications();

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
  
  //handleTelegramNotifications();

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
  //bot.sendMessage(TELEGRAM_CHAT_ID, "ESP32 Safety System is online!", "");

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
  Serial.println("Flame sensor read: " + String(digitalRead(flamePin)));
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

  display.noInverse();
  display.drawString(2,1,"Flame: ");
  display.drawString(15,1, data.flameDetected ? "YES" : "NO");
  display.drawString(2,2,"Motion: ");
  display.drawString(15,2,data.motionDetected ? "YES" : "NO");
  display.drawString(2,3,"Gas: ");
  display.drawString(15,3,data.gasValue > GAS_THRESHOLD ? "ALERT" : "OK");
  display.drawString(2,4,"Temperature: ");
  display.drawString(15,4,tempStr);
  display.inverse();
  display.drawString(2,6, alert ? "ALERT!" : "Status OK"); // first line
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
        sendCaptureCommand();
        //cameraState = CAM_WAITING_FOR_IMAGE;
        cameraState = CAM_IDLE;
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
    //send image
  }
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