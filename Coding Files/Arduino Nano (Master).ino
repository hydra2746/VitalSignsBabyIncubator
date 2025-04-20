#include <Wire.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Keypad.h>

// ---------------- Pins ------------------
#define DHTPIN 2
#define ONE_WIRE_BUS 4
const int buzzerPin = 8;
const int ledPin = 9;
const int heaterRelayPin = 6;
const int fanRelayPin = 7;

// --- Keypad Pin Definitions ---
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {13, 12, 11, 10};
byte colPins[COLS] = {A3, A2, A1, A0};
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

// ---------------- Sensor Objects ------------------
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Adafruit_MPU6050 mpu;

// ---------------- Control Parameters ------------------
const float defaultTargetIncubatorTemp = 35.0;
const float tempHysteresis = 0.5;
const float minSkinTemp = 25.0;
const float maxSkinTemp = 37.5;

// ---------------- Movement Detection ------------------
const float movementThreshold = 1.0; // Adjust sensitivity if needed
float prevX = 0, prevY = 0, prevZ = 0;
bool mpuInitialized = false; // Flag to track MPU6050 initialization

// ---------------- System State ------------------
enum OperatingMode { AUTO_MODE, WAITING_FOR_KEYPAD_TEMP, MANUAL_MODE };
OperatingMode currentMode = AUTO_MODE;
char manualInputBuffer[6]; // Increased buffer size slightly for floats like "37.5"
float manualTargetTemp = -1.0;
float currentActiveTargetTemp = defaultTargetIncubatorTemp;

// ---------------- Received MAX30100 Data ------------------
float receivedSpO2 = -1.0; // Initialize with invalid value
float receivedHR = -1.0;   // Initialize with invalid value
String serial1Buffer = ""; // Buffer for incoming serial data

// ---------------- Non-Blocking Timing (millis()) ------------------
unsigned long lastDhtReadTime = 0;
unsigned long lastDs18b20RequestTime = 0;
unsigned long lastDs18b20ReadTime = 0;
unsigned long lastMpuReadTime = 0;
unsigned long lastStatusPrintTime = 0;
unsigned long lastControlLogicTime = 0;

const unsigned long dhtInterval = 2100;         // Read DHT slightly slower than 2s
const unsigned long ds18b20RequestInterval = 1500; // How often to request DS18B20 temp
const unsigned long ds18b20ReadDelay = 800;    // Time needed for DS18B20 conversion (adjust based on resolution)
const unsigned long mpuInterval = 100;          // Read MPU6050 at 10 Hz
const unsigned long statusPrintInterval = 2000; // Print status every 2 seconds
const unsigned long controlLogicInterval = 500; // Run control logic twice per second

// ---------------- Setup ------------------
void setup() {
  Serial.begin(9600); // For Serial Monitor via USB
  while (!Serial && millis() < 5000); // Wait max 5s for Serial Monitor

  // Initialize Serial1 for communication with the regular Nano
  Serial1.begin(9600); // Baud rate MUST match the sending Nano
  serial1Buffer.reserve(64); // Pre-allocate buffer capacity

  Serial.println(F("Initializing Baby Incubator System v3.2 (Serial RX & Responsive)..."));

  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(heaterRelayPin, OUTPUT);
  pinMode(fanRelayPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledPin, LOW);
  digitalWrite(heaterRelayPin, LOW);
  digitalWrite(fanRelayPin, LOW);

  Serial.print(F("Initializing DHT11..."));
  dht.begin();
  Serial.println(F(" Done."));

  Serial.print(F("Initializing DS18B20..."));
  sensors.begin();
  Serial.println(F(" Done."));

  Serial.print(F("Initializing MPU6050..."));
  if (!mpu.begin()) {
    Serial.println(F("❌ FAILED! Check wiring. Movement detection disabled."));
    mpuInitialized = false;
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println(F("✅ Ready!"));
    mpuInitialized = true;
  }

  currentActiveTargetTemp = defaultTargetIncubatorTemp;
  manualInputBuffer[0] = '\0';
  Serial.println(F("System Initialized. Mode: AUTO"));
  Serial.println(F("Press '*' for Manual Temp, '#' to Enter, 'C' for Auto Mode."));
  delay(100); // Small delay before loop starts
}

// ---------------- Loop ------------------
void loop() {
  unsigned long currentMillis = millis(); // Get current time once per loop

  // --- 1. Check Keypad Input (Runs every loop iteration - Fast!) ---
  handleKeypad();

  // --- 2. Check for Incoming Serial Data from Regular Nano ---
  handleSerial1Input();

  // --- 3. Read Sensors (Using millis() timing) ---
  readDhtSensor(currentMillis);
  readDs18b20Sensor(currentMillis);
  readMpuSensor(currentMillis);

  // --- 4. Run Control Logic (Using millis() timing) ---
  runControlLogic(currentMillis);

  // --- 5. Print Status Update (Using millis() timing) ---
  printStatus(currentMillis);

  // --- 6. Skin Temperature & Movement Alerts (Runs frequently) ---
  //    (These checks are quick, so run them often)
  checkSkinTemperature();
  checkMovement();

  // NO MAJOR DELAY HERE - Loop runs fast, responsiveness comes from millis() checks
  // A tiny delay can sometimes help stability if needed, but start without.
  // delay(5);
} // END OF loop()

// ========================================================
// ===           Helper Functions                     ===
// ========================================================

// --- Keypad Handling Function ---
void handleKeypad() {
  char key = keypad.getKey();
  if (!key) return; // No key pressed, exit function quickly

  Serial.print(F("Key pressed: ")); Serial.println(key); // Debug

  if (currentMode == AUTO_MODE) {
    if (key == 'A') {
      currentMode = WAITING_FOR_KEYPAD_TEMP;
      manualInputBuffer[0] = '\0'; // Clear buffer
      Serial.println(F(">> Manual Mode: Enter target temp (e.g., 36.5) then '#':"));
      // Optional: Short beep/flash
    }
  } else if (currentMode == WAITING_FOR_KEYPAD_TEMP) {
    if ((key >= '0' && key <= '9') || key == '.') { // Allow digits and decimal point
      int currentLen = strlen(manualInputBuffer);
      if (currentLen < (sizeof(manualInputBuffer) - 1)) {
        manualInputBuffer[currentLen] = key;
        manualInputBuffer[currentLen + 1] = '\0';
        Serial.print(F("Input Buffer: ")); Serial.println(manualInputBuffer);
        // Optional: Short beep/flash per key
      } else {
        Serial.println(F("Buffer full. Press '#' or 'C'."));
        // Optional: Error beep/flash
      }
    } else if (key == '#') { // Enter/Confirm
      if (strlen(manualInputBuffer) > 0) {
        manualTargetTemp = atof(manualInputBuffer); // Convert buffer to float
        // Validate temperature range (adjust as needed)
        if (manualTargetTemp >= 10.0 && manualTargetTemp <= 45.0) {
          currentMode = MANUAL_MODE;
          currentActiveTargetTemp = manualTargetTemp;
          Serial.print(F(">> Manual target set: "));
          Serial.print(currentActiveTargetTemp, 1); Serial.println(F(" C"));
          Serial.println(F("Press 'C' to return to Auto mode."));
          // Optional: Confirmation beep/flash
        } else {
          Serial.println(F(">> Invalid temp range (25-45 C). Try again."));
          manualInputBuffer[0] = '\0'; // Clear buffer on error
          // Optional: Error beep/flash
        }
      } else {
         Serial.println(F(">> No temperature entered."));
         // Optional: Error beep/flash
      }
    } else if (key == 'C') { // Cancel/Return to Auto
      currentMode = AUTO_MODE;
      currentActiveTargetTemp = defaultTargetIncubatorTemp;
      manualInputBuffer[0] = '\0';
      Serial.println(F(">> Cancelled. Switched back to AUTO mode."));
      Serial.println(F("Press '*' for Manual Temp, '#' to Enter, 'C' for Auto Mode."));
      // Optional: Cancel beep/flash
    }
  } else if (currentMode == MANUAL_MODE) {
    if (key == 'C') { // Return to Auto
      currentMode = AUTO_MODE;
      currentActiveTargetTemp = defaultTargetIncubatorTemp;
      Serial.println(F(">> Switched back to AUTO mode."));
      Serial.println(F("Press '*' for Manual Temp, '#' to Enter, 'C' for Auto Mode."));
      // Optional: Confirmation beep/flash
    }
  }
  // NO DELAY needed here, keypad library handles debouncing
}

// --- Serial1 Input Handling ---
void handleSerial1Input() {
  while (Serial1.available() > 0) {
    char receivedChar = (char)Serial1.read();
    if (receivedChar == '\n') {
      // End of line/message received, process the buffer
      // Expected format: "SPO2:xx.x,HR:yy.y"
      // Serial.print("Raw Serial1 RX: "); Serial.println(serial1Buffer); // Debug

      int spo2Index = serial1Buffer.indexOf("SPO2:");
      int hrIndex = serial1Buffer.indexOf(",HR:");

      if (spo2Index != -1 && hrIndex != -1) {
          // Extract values
          String spo2Str = serial1Buffer.substring(spo2Index + 5, hrIndex);
          String hrStr = serial1Buffer.substring(hrIndex + 4);

          receivedSpO2 = spo2Str.toFloat();
          receivedHR = hrStr.toFloat();

          // Basic validation
          if (receivedSpO2 < 0 || receivedSpO2 > 100) receivedSpO2 = -2.0; // Indicate parsing/value error
          if (receivedHR < 0 || receivedHR > 250) receivedHR = -2.0;       // Indicate parsing/value error

      } else if (serial1Buffer.startsWith("ERROR:")) {
          // Handle potential error messages from the other Nano
          Serial.print("Received Error from Nano: "); Serial.println(serial1Buffer);
          receivedSpO2 = -3.0; // Indicate error received
          receivedHR = -3.0;
      }
       else {
          Serial.print("Serial1 Parse Error: "); Serial.println(serial1Buffer); // Debug
          receivedSpO2 = -1.0; // Reset to invalid on parse error
          receivedHR = -1.0;
      }

      serial1Buffer = ""; // Clear the buffer for the next message
    } else {
      // Add character to buffer, prevent overflow
      if (serial1Buffer.length() < 63) {
          serial1Buffer += receivedChar;
      } else {
          // Buffer overflow, clear it and log error
          Serial.println("Serial1 RX Buffer Overflow!");
          serial1Buffer = "";
      }
    }
  }
}


// --- Sensor Reading Functions (millis based) ---
float currentIncubatorTemp = NAN; // Use NAN for unread/error state
float currentIncubatorHumidity = NAN;
float currentSkinTemp = DEVICE_DISCONNECTED_C; // Use DS18B20 error code

void readDhtSensor(unsigned long currentMillis) {
  if (currentMillis - lastDhtReadTime >= dhtInterval) {
    lastDhtReadTime = currentMillis;
    currentIncubatorTemp = dht.readTemperature(); // Read Celsius
    currentIncubatorHumidity = dht.readHumidity();

    if (isnan(currentIncubatorTemp) || isnan(currentIncubatorHumidity)) {
      Serial.println(F("ERROR: Failed to read from DHT11 sensor!"));
      triggerSensorErrorAlarm("DHT11");
      // Keep heater/fan off if DHT fails (handled in control logic)
    }
  }
}

void readDs18b20Sensor(unsigned long currentMillis) {
  // State machine for DS18B20 reading (Request -> Wait -> Read)
  static bool waitingForDs18b20 = false;

  // Time to request a new temperature?
  if (!waitingForDs18b20 && (currentMillis - lastDs18b20RequestTime >= ds18b20RequestInterval)) {
    sensors.requestTemperatures(); // Send the command
    lastDs18b20RequestTime = currentMillis;
    waitingForDs18b20 = true;
    // Serial.println("DS18B20 Request Sent"); // Debug
  }

  // Time to read the temperature after waiting?
  if (waitingForDs18b20 && (currentMillis - lastDs18b20RequestTime >= ds18b20ReadDelay)) {
    currentSkinTemp = sensors.getTempCByIndex(0); // Read the temperature
    lastDs18b20ReadTime = currentMillis; // Record the time of successful read attempt
    waitingForDs18b20 = false;
    // Serial.print("DS18B20 Read Attempted: "); Serial.println(currentSkinTemp); // Debug

    if (currentSkinTemp == DEVICE_DISCONNECTED_C || currentSkinTemp < -50) { // Add sanity check
      Serial.println(F("ERROR: Failed to read from DS18B20 sensor!"));
      triggerSensorErrorAlarm("DS18B20");
      currentSkinTemp = DEVICE_DISCONNECTED_C; // Ensure error state
    }
  }
}

sensors_event_t accelEvent, gyroEvent, tempEvent; // MPU sensor event objects
void readMpuSensor(unsigned long currentMillis) {
  if (mpuInitialized && (currentMillis - lastMpuReadTime >= mpuInterval)) {
    lastMpuReadTime = currentMillis;
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent); // Read all events
  }
}

// --- Control Logic Function ---
void runControlLogic(unsigned long currentMillis) {
   if (currentMillis - lastControlLogicTime >= controlLogicInterval) {
     lastControlLogicTime = currentMillis;

     // Update Active Target Temp Based on Mode (redundant check, but safe)
     if (currentMode == AUTO_MODE) {
         currentActiveTargetTemp = defaultTargetIncubatorTemp;
     } else if (currentMode == MANUAL_MODE) {
         // Already set by keypad handler
     }

     // Incubator Temperature Control
     if (!isnan(currentIncubatorTemp)) {
       if (currentIncubatorTemp < (currentActiveTargetTemp - tempHysteresis)) {
         digitalWrite(heaterRelayPin, HIGH);
         digitalWrite(fanRelayPin, LOW);
         // Serial.println(F("STATUS: Heater ON")); // Reduce serial noise, print in main status
       } else if (currentIncubatorTemp > (currentActiveTargetTemp + tempHysteresis)) {
         digitalWrite(heaterRelayPin, LOW);
         digitalWrite(fanRelayPin, HIGH);
         // Serial.println(F("STATUS: Fan ON")); // Reduce serial noise
       } else {
         digitalWrite(heaterRelayPin, LOW);
         digitalWrite(fanRelayPin, LOW);
         // Serial.println(F("STATUS: Temp OK")); // Reduce serial noise
       }
     } else {
       // DHT Error - turn off climate control for safety
       digitalWrite(heaterRelayPin, LOW);
       digitalWrite(fanRelayPin, LOW);
       // Serial.println(F("STATUS: Climate Control OFF= (DHT Error)")); // Reduce serial noise
     }
   }
}


// --- Status Printing Function ---
void printStatus(unsigned long currentMillis) {
  if (currentMillis - lastStatusPrintTime >= statusPrintInterval) {
    lastStatusPrintTime = currentMillis;

    Serial.println(F("--- Status & Readings ---"));
    Serial.print(F("Mode: "));
    if (currentMode == AUTO_MODE) Serial.print(F("AUTO"));
    else if (currentMode == WAITING_FOR_KEYPAD_TEMP) Serial.print(F("WAITING_KEYPAD"));
    else if (currentMode == MANUAL_MODE) Serial.print(F("MANUAL"));
    Serial.print(F("\t Active Target: ")); Serial.print(currentActiveTargetTemp, 1); Serial.println(F(" C"));

    // Incubator Temp/Humidity
    if (!isnan(currentIncubatorTemp)) {
      Serial.print(F("Incubator: ")); Serial.print(currentIncubatorTemp, 1); Serial.print(F("C"));
      Serial.print(F(" / ")); Serial.print(currentIncubatorHumidity, 0); Serial.print(F("% RH"));
    } else {
      Serial.print(F("Incubator: --- C / --- % RH (DHT ERROR)"));
    }
    // Skin Temp
    Serial.print(F("\t Skin: "));
    if (currentSkinTemp != DEVICE_DISCONNECTED_C) {
       Serial.print(currentSkinTemp, 1); Serial.print(F(" C"));
    } else {
       Serial.print(F("--- C (DS18B20 ERROR)"));
    }
    Serial.println(); // Newline

    // SpO2/HR from MAX30100 (via Serial1)
    Serial.print(F("Oximeter: SpO2="));
    if (receivedSpO2 >= 0) {
        Serial.print(receivedSpO2, 1); Serial.print("%");
    } else if (receivedSpO2 == -1.0) {
        Serial.print("---"); // Waiting for data
    } else if (receivedSpO2 == -2.0) {
        Serial.print("ERR_VAL"); // Invalid value received
    } else if (receivedSpO2 == -3.0) {
        Serial.print("ERR_RX"); // Error message received
    } else {
        Serial.print("???");
    }

    Serial.print(F(" / HR="));
     if (receivedHR >= 0) {
        Serial.print(receivedHR, 1); Serial.print(" bpm");
    } else if (receivedHR == -1.0) {
        Serial.print("---"); // Waiting for data
    } else if (receivedHR == -2.0) {
        Serial.print("ERR_VAL"); // Invalid value received
    } else if (receivedHR == -3.0) {
        Serial.print("ERR_RX"); // Error message received
    } else {
        Serial.print("???");
    }
    Serial.println(); // Newline

    // Heater/Fan Status
    Serial.print(F("Heater: ")); Serial.print(digitalRead(heaterRelayPin) ? "ON " : "OFF");
    Serial.print(F("\t Fan: ")); Serial.println(digitalRead(fanRelayPin) ? "ON " : "OFF");

    Serial.println(F("-----------------------"));
  }
}

// --- Alert Checking Functions ---
void checkSkinTemperature() {
  // Only check if we have a valid reading
  if (currentSkinTemp != DEVICE_DISCONNECTED_C) {
     if (currentSkinTemp < minSkinTemp || currentSkinTemp > maxSkinTemp) {
       triggerTemperatureAlarm(currentSkinTemp); // Just prints for now
     }
  }
}

void checkMovement() {
  if (!mpuInitialized) return; // Don't check if MPU failed

  // Calculate change from previous reading
  float deltaX = abs(accelEvent.acceleration.x - prevX);
  float deltaY = abs(accelEvent.acceleration.y - prevY);
  float deltaZ = abs(accelEvent.acceleration.z - prevZ);
  // Update previous values ONLY if they are not zero (avoids false trigger on first read)
  if (prevX != 0 || prevY != 0 || prevZ != 0) {
      float totalChange = deltaX + deltaY + deltaZ;
      if (totalChange > movementThreshold) {
        triggerMovementAlarm(); // Sound/Light Alarm
      }
  }
  // Store current values for next comparison
  prevX = accelEvent.acceleration.x;
  prevY = accelEvent.acceleration.y;
  prevZ = accelEvent.acceleration.z;
}


// --- Alarm Trigger Functions ---
void triggerMovementAlarm() {
  Serial.println(F("🚨 ALERT: Movement Detected!"));
    for (int i=0; i<2; i++) { // Short burst
      digitalWrite(ledPin, HIGH);
      tone(buzzerPin, 1200, 150); // frequency, duration
      delay(200); // Keep small delays here for alarm pattern timing
      digitalWrite(ledPin, LOW);
      noTone(buzzerPin);
      delay(100);
  }
}

void triggerTemperatureAlarm(float currentSkinTemp) {
  // Currently just prints a message, could be expanded
  Serial.print(F("INFO: Skin temperature out of range ("));
  Serial.print(currentSkinTemp, 1);
  Serial.println(F(" C)!"));
}

void triggerSensorErrorAlarm(String sensorName) {
  // Currently just prints a message
    Serial.print(F("ERROR: Sensor Failure Detected - "));
    Serial.println(sensorName);
    // Could add LED/Buzzer pattern here if needed
}
