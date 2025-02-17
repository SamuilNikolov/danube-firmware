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
#define TELEMETRY_BUFFER_SIZE 2000  // Buffer size to accommodate full telemetry
char telemetryBuffer[TELEMETRY_BUFFER_SIZE];
int telemetryLength = 0, telemetryIndex = 0;
String cmdBuffer;  // Accumulates incoming Serial command data

// ----- Sequence Event Structure -----
// Maximum number of scheduled events is set here.
#define MAX_SEQUENCE_EVENTS 1600
struct SequenceEvent {
  // When stored, 'time' is a relative delay (in µs) from sequence start.
  // When active, it is rebased to an absolute micros() value.
  unsigned long time;
  uint8_t channel;     // Solenoid channel (1-indexed)
  uint8_t state;       // HIGH or LOW
};

// Active sequence events (used during execution)
SequenceEvent sequenceEvents[MAX_SEQUENCE_EVENTS];
int numSequenceEvents = 0;
int currentSequenceEvent = 0;
bool sequenceRunning = false;
unsigned long sequenceStartTime = 0;

// Stored sequence events (kept until "sequence" command is received)
SequenceEvent storedSequenceEvents[MAX_SEQUENCE_EVENTS];
int storedNumSequenceEvents = 0;
bool sequenceStored = false;

// ----- Build Telemetry -----
// Format: "TS:<timestamp> | ARM:<state> | BATT:<voltage>V | ARM_SENSE:<voltage>V | SOL:1:ON,2:OFF,... | SEQ:... \n"
void buildTelemetry() {
  unsigned long t = millis();
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
  
  // Append sequence info.
  int len = snprintf(telemetryBuffer + n, TELEMETRY_BUFFER_SIZE - n, " | SEQ:");
  n += len;
  if (sequenceStored) {
    if (sequenceRunning) {
      len = snprintf(telemetryBuffer + n, TELEMETRY_BUFFER_SIZE - n, "RUNNING, ");
      n += len;
    }
    for (int i = 0; i < storedNumSequenceEvents; i++) {
      len = snprintf(telemetryBuffer + n, TELEMETRY_BUFFER_SIZE - n,
                     "s%d@%lums%s", 
                     storedSequenceEvents[i].channel, 
                     storedSequenceEvents[i].time,
                     (i < storedNumSequenceEvents - 1 ? "," : ""));
      n += len;
      if(n >= TELEMETRY_BUFFER_SIZE) break;
    }
  } else {
    len = snprintf(telemetryBuffer + n, TELEMETRY_BUFFER_SIZE - n, "none");
    n += len;
  }
  
  // Append a newline if needed.
  if(n > 0 && telemetryBuffer[n-1] != '\n') {
    if(n < TELEMETRY_BUFFER_SIZE - 1) {
      telemetryBuffer[n++] = '\n';
      telemetryBuffer[n] = '\0';
    } else {
      telemetryBuffer[TELEMETRY_BUFFER_SIZE - 1] = '\n';
      n = TELEMETRY_BUFFER_SIZE;
    }
  }
  
  telemetryLength = n;
  telemetryIndex = 0;
}

// When a "sequence" command is received, start execution of the stored sequence.
void startSequence() {
  sequenceStartTime = micros();
  // Rebase all stored events to absolute times.
  for (int i = 0; i < storedNumSequenceEvents; i++) {
    sequenceEvents[i].channel = storedSequenceEvents[i].channel;
    sequenceEvents[i].state   = storedSequenceEvents[i].state;
    sequenceEvents[i].time    = sequenceStartTime + storedSequenceEvents[i].time;
  }
  numSequenceEvents = storedNumSequenceEvents;
  currentSequenceEvent = 0;
  sequenceRunning = true;
}

// ----- Process Command -----
// Commands:
//   - "command:..." → Store a sequence (with events and delays given in µs)
//   - "sequence"    → Launch the stored sequence (if one exists)
//   - "abort"       → Stop any running sequence and clear the stored sequence
//   - "a", "d", "e", or direct "sXY" commands operate as before.
void processCommand(const String &cmd) {
  String command = cmd;
  command.trim();
  if (command.length() == 0)
    return;
  
  // "abort" command: stop the sequence and clear stored sequence info.
  if (command.equalsIgnoreCase("abort")) {
    sequenceRunning = false;
    sequenceStored = false;
    storedNumSequenceEvents = 0;
    numSequenceEvents = 0;
    currentSequenceEvent = 0;
    return;
  }
  
  // If the command is "sequence", trigger the stored sequence.
  if (command.equalsIgnoreCase("sequence")) {
    if (sequenceStored) {
      startSequence();
    }
    return;
  }
  
  // ----- Sequence Command Storage Handling -----
  if (command.startsWith("command:")) {
    // Remove the prefix.
    String seqStr = command.substring(8);
    // Split by '.' into tokens.
    const int maxTokens = 2000;
    String tokens[maxTokens];
    int tokenCount = 0;
    int startIndex = 0;
    int dotIndex = seqStr.indexOf('.');
    while (dotIndex != -1 && tokenCount < maxTokens) {
      tokens[tokenCount++] = seqStr.substring(startIndex, dotIndex);
      startIndex = dotIndex + 1;
      dotIndex = seqStr.indexOf('.', startIndex);
    }
    if (tokenCount < maxTokens)
      tokens[tokenCount++] = seqStr.substring(startIndex);
      
    // The format should be an odd number of tokens:
    // event, delay, event, delay, …, event.
    if (tokenCount % 2 == 0) {
      // Invalid format.
      return;
    }
    
    // Reset stored sequence.
    storedNumSequenceEvents = 0;
    unsigned long relativeTime = 0;  // Relative delay in µs.
    
    // Process tokens:
    // Even-numbered tokens are events (e.g., "s11" means solenoid 1 ON).
    // Odd-numbered tokens are delay intervals (in µs).
    for (int i = 0; i < tokenCount; i++) {
      if (i % 2 == 0) {
        // Event token.
        String token = tokens[i];
        token.trim();
        if (token.length() < 2) continue;
        if (token.charAt(0) == 's' || token.charAt(0) == 'S') {
          // The last character indicates state; the rest (after 's') is the channel.
          String channelStr = token.substring(1, token.length() - 1);
          int channel = channelStr.toInt();
          char stateChar = token.charAt(token.length() - 1);
          int state = (stateChar == '1') ? HIGH : LOW;
          if (channel >= 1 && channel <= numSolenoids && storedNumSequenceEvents < MAX_SEQUENCE_EVENTS) {
            storedSequenceEvents[storedNumSequenceEvents].time = relativeTime;
            storedSequenceEvents[storedNumSequenceEvents].channel = channel;
            storedSequenceEvents[storedNumSequenceEvents].state = state;
            storedNumSequenceEvents++;
          }
        }
      } else {
        // Delay token.
        unsigned long delayVal = tokens[i].toInt();
        relativeTime += delayVal;
      }
    }
    
    if (storedNumSequenceEvents > 0) {
      sequenceStored = true;
    }
    return;
  }
  
  // ----- Other Commands -----
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
  if (command.equalsIgnoreCase("e")) {
    sequenceRunning = false;
    sequenceStored = false;
    storedNumSequenceEvents = 0;
    numSequenceEvents = 0;
    currentSequenceEvent = 0;
    for (int i = 0; i < numSolenoids; i++) {
      pinMode(solenoidPins[i], OUTPUT);
      digitalWrite(solenoidPins[i], LOW);
      solenoidStates[i] = false;
    }
    digitalWrite(ARM_PIN, LOW);
    digitalWrite(DISARM_PIN, HIGH);
    return;
  }
  
  // Direct solenoid actuation (e.g., "s11" to set solenoid 1 HIGH).
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
  Serial3.begin(2000000);
  
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
  pinMode(41, OUTPUT);
  digitalWrite(41, HIGH);
  
  // Build the first telemetry message.
  buildTelemetry();
}

void loop() {
  // --- Non-blocking Command Reception ---
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
  if (telemetryIndex >= telemetryLength) {
    buildTelemetry();
  }
  
  // --- Non-blocking Sequence Execution ---
  if (sequenceRunning) {
    unsigned long now = micros();
    // Fire any events whose scheduled time has arrived.
    while (currentSequenceEvent < numSequenceEvents &&
           (long)(now - sequenceEvents[currentSequenceEvent].time) >= 0) {
      int channel = sequenceEvents[currentSequenceEvent].channel;
      int state   = sequenceEvents[currentSequenceEvent].state;
      digitalWrite(solenoidPins[channel - 1], state);
      solenoidStates[channel - 1] = (state == HIGH);
      currentSequenceEvent++;
    }
    if (currentSequenceEvent >= numSequenceEvents) {
      sequenceRunning = false;
    }
  }
}
