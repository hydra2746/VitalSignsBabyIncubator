#include <Wire.h>
#include <SoftwareSerial.h>
#include "MAX30100_PulseOximeter.h" // Use the correct header for your library

// --- Software Serial Pins for Arduino 1 ---
const byte ARDUINO1_RX_PIN = 4; // Connect this to Arduino 1's TX pin (Pin 5 in the example)
const byte ARDUINO1_TX_PIN = 5; // Connect this to Arduino 1's RX pin (Pin 4 in the example)
SoftwareSerial arduino1Serial(ARDUINO1_RX_PIN, ARDUINO1_TX_PIN); // RX, TX

// --- MAX30100 Setup ---
PulseOximeter pox;
// If using Sparkfun library, uncomment this when creating the object:
// #define REPORTING_PERIOD_MS     1000
// uint32_t tsLastReport = 0;

// Variables to store sensor readings
float heartRate;
float spO2;

void setup() {
  Serial.begin(9600); // For local debugging (optional)
  Serial.println("Initializing MAX30100 Sensor...");

  arduino1Serial.begin(9600); // Start communication with Arduino 1

  // Initialize the MAX30100 sensor
  if (!pox.begin()) {
    Serial.println("ERROR: Failed to initialize MAX30100 sensor!");
    while (1); // Stop execution
  } else {
    Serial.println("MAX30100 sensor initialized.");
  }

  // Optional: Configure the sensor (LED currents, sample rates, etc.)
  // Example using Sparkfun library (adjust values as needed)
  // pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA); // Example: Set IR LED current
  // pox.setRedLedCurrent(MAX30100_LED_CURR_27_1MA); // Example: Set Red LED current

  Serial.println("Ready to send data...");
}

void loop() {
  // Update the sensor readings
  // For Sparkfun library, you might need something like: pox.update();
  // Or other libraries might read directly:
  pox.update(); // Call update function from library if needed

  // Get readings (adapt function names based on your library)
  heartRate = pox.getHeartRate();
  spO2 = pox.getSpO2();

  // Check if valid readings are available (some libraries might have a check function)
  // Example check (adjust threshold as needed, >0 usually means valid for Sparkfun)
  if (heartRate > 0 && spO2 > 0) {

    // Format data as "HR,SpO2"
    String dataToSend = String(heartRate, 1) + "," + String(spO2, 1); // Send with 1 decimal place

    // Send data to Arduino 1 via SoftwareSerial
    arduino1Serial.println(dataToSend);

    // Optional: Print locally for debugging
    Serial.print("Sent: ");
    Serial.println(dataToSend);

  } else {
     // Optional: Print local message if no finger detected or readings invalid
     Serial.println("No valid readings / Place finger on sensor");
     // You might choose to send default values like "0.0,0.0" or nothing
     // arduino1Serial.println("0.0,0.0");
  }


  // Wait before sending the next reading
  // Adjust delay based on how frequently you need updates and sensor settling time
  delay(1000); // Send data every 1 second
}