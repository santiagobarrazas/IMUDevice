#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>
#include <Wire.h>
#include <ArduinoJson.h>

const int DEVICE_TYPE = 2;

BLEService imuService("12345678-1234-5678-1234-56789abcdef0");
BLECharacteristic imuDataCharacteristic("12345678-1234-5678-1234-56789abcdef1", BLERead | BLENotify, 200); // Increased buffer size
BLECharacteristic commandCharacteristic("12345678-1234-5678-1234-56789abcdef2", BLERead | BLEWrite, 1);

bool isCapturing = false;

const char* DEVICE_NAMES[] = {
  "LEFT-HAND",
  "RIGHT-HAND",
  "LEFT-ANKLE",
  "RIGHT-ANKLE",
  "BASE-SPINE"
};

void onCommandWritten(BLEDevice central, BLECharacteristic characteristic) {
  if (characteristic.uuid() == commandCharacteristic.uuid()) {
    uint8_t command = *characteristic.value();
    isCapturing = (command == 1);
    digitalWrite(LED_BUILTIN, isCapturing);
    Serial.print("Capture state changed to: ");
    Serial.println(isCapturing);
    Serial.print("Command received: ");
    Serial.println(command);  // Add this log
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  if (!BLE.begin()) {
    Serial.println("BLE failed!");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  if (!IMU.begin()) {
    Serial.println("IMU failed!");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }
  }

  // Print IMU settings
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");

  // Configure BLE
  BLE.setLocalName(DEVICE_NAMES[DEVICE_TYPE]);
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(imuDataCharacteristic);
  imuService.addCharacteristic(commandCharacteristic);
  BLE.addService(imuService);

  commandCharacteristic.setEventHandler(BLEWritten, onCommandWritten);

  BLE.advertise();
  Serial.println("BLE device active, waiting for connections...");
}

void loop() {
  BLEDevice central = BLE.central();
  
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    
    while (central.connected()) {
      if (isCapturing) {
        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
          float accelX, accelY, accelZ;
          float gyroX, gyroY, gyroZ;

          IMU.readAcceleration(accelX, accelY, accelZ);
          IMU.readGyroscope(gyroX, gyroY, gyroZ);

          StaticJsonDocument<200> jsonDoc;
          jsonDoc["deviceId"] = DEVICE_NAMES[DEVICE_TYPE];
          jsonDoc["timestamp"] = millis();

          JsonObject accel = jsonDoc.createNestedObject("accelerometer");
          accel["x"] = accelX;
          accel["y"] = accelY;
          accel["z"] = accelZ;

          JsonObject gyro = jsonDoc.createNestedObject("gyroscope");
          gyro["x"] = gyroX;
          gyro["y"] = gyroY;
          gyro["z"] = gyroZ;

          String jsonString;
          serializeJson(jsonDoc, jsonString);
          
          Serial.println(jsonString); // Debug print
          
          if (imuDataCharacteristic.writeValue(jsonString.c_str(), jsonString.length())) {
            Serial.println("Data sent successfully");
          } else {
            Serial.println("Failed to send data");
          }
        }
        delay(20); // 50Hz sampling rate
      }
      BLE.poll();
    }
    Serial.println("Disconnected from central");
  }
}