#include <Arduino.h>
#include <Arduino_FreeRTOS.h>  // For multithreading support (or use TeensyThreads if you prefer)

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
const unsigned long telemetryInterval = 100;  // You can try reducing this—but be sure to test the impact on Serial throughput

// Buffer for incoming commands from Serial3 (shared by the command processing task)
String commandBuffer = "";


// ----- Command Processing Function -----
// This function parses a command string and performs the appropriate action.
void processCommand(String cmd) {
  cmd.trim();  // Remove any leading/trailing whitespace
  if (cmd.length() == 0) return;  // Ignore empty commands

  // --- Process Internal Arm/Disarm Commands ---
  // Use "a" for arm and "d" for disarm.
  if (cmd.equalsIgnoreCase("a")) {
    digitalWrite(ARM_PIN, HIGH);
    digitalWrite(DISARM_PIN, LOW);
    internalArmed = true;
    return;
  }
  if (cmd.equalsIgnoreCase("d")) {
    digitalWrite(ARM_PIN, LOW);
    digitalWrite(DISARM_PIN, HIGH);
    internalArmed = false;
    return;
  }

  // --- Process Solenoid Actuation Commands ---
  // Command format: s{channel}{state}
  // Examples: "s10" means channel 1 → state 0; "s51" means channel 5 → state 1.
  if (cmd.charAt(0) == 's' || cmd.charAt(0) == 'S') {
    char stateChar = cmd.charAt(cmd.length() - 1);
    int stateVal = (stateChar == '1') ? HIGH : LOW;
    String channelStr = cmd.substring(1, cmd.length() - 1);
    int channel = channelStr.toInt();
    if (channel >= 1 && channel <= numSolenoids) {
      int pin = solenoidPins[channel - 1];
      digitalWrite(pin, stateVal);
      solenoidStates[channel - 1] = (stateVal == HIGH);
    }
    return;
  }

  // (Optional) For unrecognized commands, you might send an error message.
  // Serial3.println("ERROR: Unrecognized command: " + cmd);
}


// ----- Telemetry Function -----
// Reads sensor data and system states, builds a telemetry string, and sends it over Serial3.
void sendTelemetry() {
  unsigned long timestamp = millis();
  int adcBattery = analogRead(BATT_SENSE_PIN);
  float batteryVoltage = (adcBattery / 4095.0) * 3.3;
  int adcArming = analogRead(ARMING_SENSE_PIN);
  float armingVoltage = (adcArming / 4095.0) * 3.3;

  // Build a telemetry string
  String telemetry = "";
  telemetry += "TS:" + String(timestamp);
  telemetry += " | ARM:" + String(internalArmed ? "ARMED" : "DISARMED");
  telemetry += " | BATT:" + String(batteryVoltage, 2) + "V";
  telemetry += " | ARM_SENSE:" + String(armingVoltage, 2) + "V";
  telemetry += " | SOL:";
  for (int i = 0; i < numSolenoids; i++) {
    telemetry += String(i + 1) + (solenoidStates[i] ? ":ON" : ":OFF");
    if (i < numSolenoids - 1) telemetry += ",";
  }

  // Transmit telemetry. (Note: Serial3.println() will block if the UART buffer is full.)
  Serial3.println(telemetry);
}


// ----- Telemetry Task -----
// This task runs in its own thread and transmits telemetry at the defined interval.
void TelemetryTask(void *pvParameters) {
  (void) pvParameters;
  // Use vTaskDelayUntil() to achieve a periodic (non-blocking) delay
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    sendTelemetry();
    vTaskDelayUntil(&xLastWakeTime, telemetryInterval / portTICK_PERIOD_MS);
  }
}


// ----- Command Processing Task -----
// This task continuously reads from Serial3 and processes incoming commands.
void CommandTask(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    while (Serial3.available() > 0) {
      char inChar = (char)Serial3.read();
      if (inChar == '\n' || inChar == '\r') {
        if (commandBuffer.length() > 0) {
          processCommand(commandBuffer);
          commandBuffer = "";
        }
      } else {
        commandBuffer += inChar;
      }
    }
    // Yield to allow other tasks (like TelemetryTask) to run
    vTaskDelay(1);
  }
}


// ----- Setup Function -----
void setup() {
  // Initialize Serial3 for communication with the ground station
  Serial3.begin(115200);

  // --- RS-485 Bus Setup ---
  pinMode(RS485_1_DE, OUTPUT);
  digitalWrite(RS485_1_DE, HIGH);
  pinMode(RS485_2_DE, OUTPUT);
  digitalWrite(RS485_2_DE, HIGH);

  // --- Internal Arm/Disarm Setup ---
  pinMode(ARM_PIN, OUTPUT);
  pinMode(DISARM_PIN, OUTPUT);
  digitalWrite(ARM_PIN, LOW);
  digitalWrite(DISARM_PIN, HIGH);
  internalArmed = false;

  // --- Solenoid Pins Setup ---
  for (int i = 0; i < numSolenoids; i++) {
    pinMode(solenoidPins[i], OUTPUT);
    digitalWrite(solenoidPins[i], LOW);
    solenoidStates[i] = false;
  }

  // --- Voltage Sensing Pins ---
  pinMode(BATT_SENSE_PIN, INPUT);
  pinMode(ARMING_SENSE_PIN, INPUT);

  // Create FreeRTOS tasks:
  // The CommandTask is given a slightly higher priority to ensure rapid command processing.
  xTaskCreate(CommandTask, "Command", 256, NULL, 2, NULL);
  xTaskCreate(TelemetryTask, "Telemetry", 256, NULL, 1, NULL);
}


// ----- Main Loop -----
// In a FreeRTOS-based design, loop() can remain empty because the tasks run independently.
void loop() {
  // Nothing needed here—the tasks run concurrently.
}
