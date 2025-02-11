#include <Arduino.h>

// ----- Pin Assignments -----
// Solenoid channel pins (channels 1..16):
const int numSolenoids = 16;
const int solenoidPins[numSolenoids] = {
  40, 39, 38, 37, 36, 35, 34, 33,  // Channels 1-8
  7, 6, 5, 4, 3, 2, 1, 0           // Channels 9-16
};

// RS-485 Bus 1:
const int RS485_1_RX = 15;      // Not used in this firmware
const int RS485_1_TX = 14;      // Not used in this firmware
const int RS485_1_DE = 41;      // Driver Enable (tied HIGH)

// RS-485 Bus 2:
const int RS485_2_RX = 16;      // Not used in this firmware
const int RS485_2_TX = 17;      // Not used in this firmware
const int RS485_2_DE = 18;      // Driver Enable (tied HIGH)

// Internal arming lines:
const int ARM_PIN    = 20;
const int DISARM_PIN = 21;

// Voltage sensing pins:
const int BATT_SENSE_PIN   = 23;  // Battery voltage
const int ARMING_SENSE_PIN = 22;  // Arming voltage sense

// ----- Global State Variables -----
bool solenoidStates[numSolenoids] = { false };  // false = de-energized (LOW), true = energized (HIGH)
bool internalArmed = false;   // false => Disarmed, true => Armed

// Telemetry update interval (in milliseconds)
const unsigned long telemetryInterval = 100;
unsigned long lastTelemetryTime = 0;

// Buffer for incoming commands from Serial3
String commandBuffer = "";

// ----- Setup Function -----
void setup() {
  // Initialize Serial3 for communication with the ground station
  Serial3.begin(115200);

  // --- RS-485 Bus Setup ---
  pinMode(RS485_1_DE, OUTPUT);
  digitalWrite(RS485_1_DE, HIGH);  // Hold DE high permanently

  pinMode(RS485_2_DE, OUTPUT);
  digitalWrite(RS485_2_DE, HIGH);  // Hold DE high permanently

  // --- Internal Arm/Disarm Setup ---
  pinMode(ARM_PIN, OUTPUT);
  pinMode(DISARM_PIN, OUTPUT);
  // Default state: Disarmed (set DISARM high, ARM low)
  digitalWrite(ARM_PIN, LOW);
  digitalWrite(DISARM_PIN, HIGH);
  internalArmed = false;

  // --- Solenoid Pins Setup ---
  for (int i = 0; i < numSolenoids; i++) {
    pinMode(solenoidPins[i], OUTPUT);
    digitalWrite(solenoidPins[i], LOW);  // Default: de-energized
    solenoidStates[i] = false;
  }

  // --- Voltage Sensing Pins ---
  pinMode(BATT_SENSE_PIN, INPUT);
  pinMode(ARMING_SENSE_PIN, INPUT);

  // (Optional) Initialize onboard Serial for debugging.
  // Serial.begin(115200);
}

// ----- Command Processing -----
// This function parses an individual command string and acts accordingly.
void processCommand(String cmd) {
  cmd.trim();  // Remove any leading/trailing whitespace
  if (cmd.length() == 0) return;  // Ignore empty commands

  // --- Process Internal Arm/Disarm Commands ---
  // Use "a" for arm and "d" for disarm.
  if (cmd.equalsIgnoreCase("a")) {
    // Internal ARM: set ARM_PIN HIGH and DISARM_PIN LOW
    digitalWrite(ARM_PIN, HIGH);
    digitalWrite(DISARM_PIN, LOW);
    internalArmed = true;
    return;
  }
  if (cmd.equalsIgnoreCase("d")) {
    // Internal DISARM: set DISARM_PIN HIGH and ARM_PIN LOW
    digitalWrite(ARM_PIN, LOW);
    digitalWrite(DISARM_PIN, HIGH);
    internalArmed = false;
    return;
  }

  // --- Process Solenoid Actuation Commands ---
  // Command format: s{channel}{state}
  // Examples: "s10" means channel 1 → state 0; "s51" means channel 5 → state 1.
  if (cmd.charAt(0) == 's' || cmd.charAt(0) == 'S') {
    // The last character is the state ('0' or '1').
    char stateChar = cmd.charAt(cmd.length() - 1);
    int stateVal = (stateChar == '1') ? HIGH : LOW;

    // The characters between the 's' and the final digit represent the solenoid channel number.
    String channelStr = cmd.substring(1, cmd.length() - 1);
    int channel = channelStr.toInt();

    // Check that the channel is within bounds (channels 1 to 16).
    if (channel >= 1 && channel <= numSolenoids) {
      int pin = solenoidPins[channel - 1];
      digitalWrite(pin, stateVal);
      solenoidStates[channel - 1] = (stateVal == HIGH);
    }
    return;
  }

  // (Optional) If the command is unrecognized, you could send an error response.
  // Serial3.println("ERROR: Unrecognized command: " + cmd);
}

// ----- Telemetry Function -----
// This function reads the current system states and sends a telemetry line over Serial3.
void sendTelemetry() {
  unsigned long timestamp = millis();

  // Read the battery voltage (using analogRead on BATT_SENSE_PIN)
  int adcBattery = analogRead(BATT_SENSE_PIN);
  // Assuming a 12-bit ADC (0-4095) and a 3.3V reference.
  float batteryVoltage = (adcBattery / 4095.0) * 3.3;

  // Read the arming sense voltage (using analogRead on ARMING_SENSE_PIN)
  int adcArming = analogRead(ARMING_SENSE_PIN);
  float armingVoltage = (adcArming / 4095.0) * 3.3;

  // Build a telemetry string.
  String telemetry = "";
  telemetry += "TS:" + String(timestamp);
  telemetry += " | ARM:" + String(internalArmed ? "ARMED" : "DISARMED");
  telemetry += " | BATT:" + String(batteryVoltage, 2) + "V";
  telemetry += " | ARM_SENSE:" + String(armingVoltage, 2) + "V";
  telemetry += " | SOL:";

  // Append solenoid states in the format "channel:ON/OFF" separated by commas.
  for (int i = 0; i < numSolenoids; i++) {
    telemetry += String(i + 1) + (solenoidStates[i] ? ":ON" : ":OFF");
    if (i < numSolenoids - 1) {
      telemetry += ",";
    }
  }

  // Send the telemetry line over Serial3.
  Serial3.println(telemetry);
}

// ----- Main Loop -----
void loop() {
  // --- Process Incoming Commands from Serial3 ---
  while (Serial3.available() > 0) {
    char inChar = (char)Serial3.read();
    // Look for newline or carriage return as command terminators.
    if (inChar == '\n' || inChar == '\r') {
      if (commandBuffer.length() > 0) {
        processCommand(commandBuffer);
        commandBuffer = "";
      }
    } else {
      commandBuffer += inChar;
    }
  }

  // --- Non-Blocking Telemetry Update ---
  unsigned long currentMillis = millis();
  if (currentMillis - lastTelemetryTime >= telemetryInterval) {
    sendTelemetry();
    lastTelemetryTime = currentMillis;
  }
}
