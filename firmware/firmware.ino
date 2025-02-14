#include <Arduino.h>

// ----- Pin Assignments -----
const int ARM_PIN    = 20;
const int DISARM_PIN = 21;

// Solenoid channels 1..16.
const int numSolenoids = 16;
const int solenoidPins[numSolenoids] = {
  40, 39, 38, 37, 36, 35, 34, 33,  // Channels 1-8
  7,  6,  5,  4,  3,  2,  1,  0    // Channels 9-16
};
bool solenoidStates[numSolenoids] = { false };

// Voltage sensing pins.
const int BATT_SENSE_PIN   = 23;  // Battery voltage
const int ARMING_SENSE_PIN = 22;  // Arming voltage sense

// ----- Telemetry Buffer & Command Buffer -----
#define TELEMETRY_BUFFER_SIZE 128  // Buffer size to accommodate full telemetry
char telemetryBuffer[TELEMETRY_BUFFER_SIZE];
int telemetryLength = 0, telemetryIndex = 0;

String cmdBuffer;  // Accumulates incoming Serial3 command data

// ----- Build Telemetry -----
// Format: "TS:<timestamp> | ARM:<state> | BATT:<voltage>V | ARM_SENSE:<voltage>V | SOL:1:ON,2:OFF,...\n"
void buildTelemetry() {
  unsigned long t = millis();
  // Read analog values and convert (adjust conversion factors as needed)
  int adcBattery = analogRead(BATT_SENSE_PIN);
  float batteryVoltage = adcBattery / 28.5;
  int adcArming = analogRead(ARMING_SENSE_PIN);
  float armingVoltage = adcArming / 197.0;
  
  int n = snprintf(telemetryBuffer, TELEMETRY_BUFFER_SIZE,
                   "TS:%lu | ARM:%d | BATT:%.2fV | ARM_SENSE:%.2fV | SOL:",
                   t, digitalRead(ARM_PIN), batteryVoltage, armingVoltage);
  
  // Append solenoid states (channels 1 to 16)
  for (int i = 0; i < numSolenoids; i++) {
    int written = snprintf(telemetryBuffer + n, TELEMETRY_BUFFER_SIZE - n,
                             "%d:%s%s", i + 1, solenoidStates[i] ? "ON" : "OFF",
                             (i < numSolenoids - 1) ? "," : "");
    n += written;
    if(n >= TELEMETRY_BUFFER_SIZE) break;
  }
  
  // Ensure newline is appended.
  if(n > 0 && telemetryBuffer[n-1] != '\n') {
    if(n < TELEMETRY_BUFFER_SIZE - 1) {
      telemetryBuffer[n++] = '\n';
      telemetryBuffer[n] = '\0';
    } else {
      // No room to append, so overwrite the last character with '\n'
      telemetryBuffer[TELEMETRY_BUFFER_SIZE - 1] = '\n';
      n = TELEMETRY_BUFFER_SIZE;
    }
  }
  
  telemetryLength = n;
  telemetryIndex = 0;
}

// ----- Process Command -----
// Commands:
//    "a" → arm (ARM_PIN HIGH, DISARM_PIN LOW)
//    "d" → disarm (ARM_PIN LOW, DISARM_PIN HIGH)
//    "s{channel}{state}" → set solenoid (channel: 1–16; state: '1' for HIGH, '0' for LOW)
void processCommand(const String &cmd) {
  String command = cmd;
  command.trim();
  if (command.length() == 0)
    return;
  
  // Arm/disarm commands.
  if (command.equalsIgnoreCase("a")) {
    digitalWrite(ARM_PIN, HIGH);
    digitalWrite(DISARM_PIN, LOW);
    return;
  }
  if (command.equalsIgnoreCase("d")) {
    digitalWrite(ARM_PIN, LOW);
    digitalWrite(DISARM_PIN, HIGH);
    return;
  }
  
  // Solenoid actuation commands.
  if (tolower(command.charAt(0)) == 's') {
    if (command.length() >= 2) {
      char stateChar = command.charAt(command.length() - 1);
      int stateVal = (stateChar == '1') ? HIGH : LOW;
      String channelStr = command.substring(1, command.length() - 1);
      int channel = channelStr.toInt();
      if (channel >= 1 && channel <= numSolenoids) {
        digitalWrite(solenoidPins[channel - 1], stateVal);
        solenoidStates[channel - 1] = (stateVal == HIGH);
      }
    }
    return;
  }
}

void setup() {
  Serial3.begin(115200);
  
  // Initialize arm/disarm pins.
  pinMode(ARM_PIN, OUTPUT);
  pinMode(DISARM_PIN, OUTPUT);
  digitalWrite(ARM_PIN, LOW);    // Default: disarmed
  digitalWrite(DISARM_PIN, HIGH);
  
  // Initialize solenoid pins.
  for (int i = 0; i < numSolenoids; i++) {
    pinMode(solenoidPins[i], OUTPUT);
    digitalWrite(solenoidPins[i], LOW);
    solenoidStates[i] = false;
  }
  
  // Initialize voltage sensing pins.
  pinMode(BATT_SENSE_PIN, INPUT);
  pinMode(ARMING_SENSE_PIN, INPUT);
  
  // Build the first telemetry message.
  buildTelemetry();
}

void loop() {
  // --- Non-blocking Command Processing ---
  while (Serial3.available() > 0) {
    char c = Serial3.read();
    if (c == '\n' || c == '\r') {
      if (cmdBuffer.length() > 0) {
        processCommand(cmdBuffer);
        cmdBuffer = "";
      }
    } else {
      cmdBuffer += c;
    }
  }
  
  // --- Non-blocking Telemetry Transmission ---
  if (telemetryIndex < telemetryLength) {
    int chunk = min(8, telemetryLength - telemetryIndex);
    if (Serial3.availableForWrite() >= chunk) {
      Serial3.write(telemetryBuffer + telemetryIndex, chunk);
      telemetryIndex += chunk;
    }
  }
  
  // Once the full telemetry message is sent, build a new one.
  if (telemetryIndex >= telemetryLength) {
    buildTelemetry();
  }
}
