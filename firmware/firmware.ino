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
const int BATT_SENSE_PIN   = 23;  // Battery voltage sensor
const int ARMING_SENSE_PIN = 22;  // Arming voltage sensor

// ----- Telemetry & Command Buffer -----
#define TELEMETRY_BUFFER_SIZE 2000  
char telemetryBuffer[TELEMETRY_BUFFER_SIZE];
int telemetryLength = 0, telemetryIndex = 0;
String cmdBuffer;  // Accumulates command data received via Serial

// ----- Conditional Sequencing Structures -----
//
// StepType defines the three types of steps.
enum StepType { STEP_EVENT, STEP_DELAY, STEP_CONDITIONAL };

// Forward declaration of SequenceStep.
struct SequenceStep;

// The SequenceStep structure represents a single step in the sequence.
struct SequenceStep {
  StepType type;               // Type of step (EVENT, DELAY, or CONDITIONAL)
  unsigned long timeOffset;    // For EVENT and DELAY: delay (in µs) relative to branch start
  
  // For EVENT steps:
  uint8_t channel;             // Solenoid channel (1-indexed)
  uint8_t state;               // HIGH or LOW
  
  // For CONDITIONAL steps:
  String condition;            // Condition string (e.g., "ARMING>3")
  unsigned long timeout;       // Timeout in µs for the condition
  
  // Branch arrays for conditionals (using pointers to avoid recursive embedded arrays)
  SequenceStep* trueBranch;    // Array of steps to execute if condition is met
  int trueBranchCount;         // Number of steps in the true branch
  
  SequenceStep* falseBranch;   // Array of steps to execute if condition times out
  int falseBranchCount;        // Number of steps in the false branch
};

// The root sequence is stored as an array of SequenceStep.
#define MAX_ROOT_STEPS 100
SequenceStep rootSequence[MAX_ROOT_STEPS];
int rootSequenceCount = 0;

// ----- Memory Cleanup Function -----
//
// This function recursively frees memory allocated for branch arrays.
void freeSequenceSteps(SequenceStep steps[], int count) {
  for (int i = 0; i < count; i++) {
    if (steps[i].type == STEP_CONDITIONAL) {
      // If the true branch exists, free it recursively and delete the allocated array.
      if (steps[i].trueBranch != NULL) {
        freeSequenceSteps(steps[i].trueBranch, steps[i].trueBranchCount);
        delete[] steps[i].trueBranch;
        steps[i].trueBranch = NULL;
        steps[i].trueBranchCount = 0;
      }
      // Similarly for the false branch.
      if (steps[i].falseBranch != NULL) {
        freeSequenceSteps(steps[i].falseBranch, steps[i].falseBranchCount);
        delete[] steps[i].falseBranch;
        steps[i].falseBranch = NULL;
        steps[i].falseBranchCount = 0;
      }
    }
  }
}

// ----- Parsing Functions -----
//
// The parser converts a command string into a tree of SequenceStep items.
// The grammar supported is as follows:
//
//   Sequence  := Step { '.' Step }*
//   Step      := Event | Delay | Conditional
//   Event     := 's' <channel> <state>
//   Delay     := <number>
//   Conditional := "if(" <condition> "," <trueSequence> "," <timeout> ":" <falseSequence> ")"
//   (Branches in a conditional use commas as separators.)
//
// Helper function: parseUntil reads characters until a given delimiter is found.
String parseUntil(const String &s, int &pos, char delimiter) {
  int start = pos;
  while (pos < s.length() && s.charAt(pos) != delimiter) {
    pos++;
  }
  String token = s.substring(start, pos);
  if (pos < s.length() && s.charAt(pos) == delimiter) { pos++; }
  return token;
}

// Forward declaration for parseStep.
SequenceStep parseStep(const String &s, int &pos, char separator);

// parseSequence parses multiple steps (a branch) using a specified separator.
int parseSequence(const String &s, int &pos, SequenceStep steps[], int maxSteps, char separator) {
  int count = 0;
  // Parse until a closing parenthesis, newline, or end of string is encountered.
  while (pos < s.length() && s.charAt(pos) != ')' && s.charAt(pos) != '\n') {
    if (s.charAt(pos) == separator) { pos++; continue; }
    if (count >= maxSteps) break;
    steps[count] = parseStep(s, pos, separator);
    count++;
    if (pos < s.length() && s.charAt(pos) == separator) { pos++; }
  }
  return count;
}

// parseStep parses a single step.
SequenceStep parseStep(const String &s, int &pos, char separator) {
  SequenceStep step;
  // Initialize default values.
  step.timeOffset = 0;
  step.channel = 0;
  step.state = 0;
  step.condition = "";
  step.timeout = 0;
  step.trueBranch = NULL;
  step.trueBranchCount = 0;
  step.falseBranch = NULL;
  step.falseBranchCount = 0;
  
  // Check if the step is a conditional (begins with "if(").
  if (s.startsWith("if(", pos)) {
    pos += 3; // Skip "if("
    step.type = STEP_CONDITIONAL;
    // Parse the condition up to the first comma.
    step.condition = parseUntil(s, pos, ',');
    // Allocate branch arrays (fixed size of 50 steps per branch).
    step.trueBranch = new SequenceStep[50];
    step.falseBranch = new SequenceStep[50];
    // Parse the true branch steps, using commas as separators.
    step.trueBranchCount = parseSequence(s, pos, step.trueBranch, 50, ',');
    // Parse the timeout value until ':' is encountered.
    String timeoutStr = parseUntil(s, pos, ':');
    step.timeout = timeoutStr.toInt();
    // Parse the false branch steps until the closing ')' is reached.
    step.falseBranchCount = parseSequence(s, pos, step.falseBranch, 50, ',');
    if (pos < s.length() && s.charAt(pos) == ')') { pos++; }
  } else {
    // If not a conditional, determine if it is an event or a delay.
    char c = s.charAt(pos);
    if (c == 's' || c == 'S') {
      // EVENT step: format "s<channel><state>"
      step.type = STEP_EVENT;
      pos++; // Skip the 's'
      int start = pos;
      // Read until a separator or delimiter.
      while (pos < s.length() && s.charAt(pos) != separator && s.charAt(pos) != '\n' && s.charAt(pos) != '.') {
        pos++;
      }
      String token = s.substring(start, pos);
      if (token.length() >= 2) {
        String channelStr = token.substring(0, token.length()-1);
        step.channel = channelStr.toInt();
        char stateChar = token.charAt(token.length()-1);
        step.state = (stateChar == '1') ? HIGH : LOW;
      }
    } else {
      // Otherwise, assume the step is a delay (a number).
      step.type = STEP_DELAY;
      String numStr = parseUntil(s, pos, separator);
      step.timeOffset = numStr.toInt();
    }
  }
  
  return step;
}

// ----- Execution Engine -----
//
// This engine processes the sequence tree nonblocking using a branch stack.
// Each branch represents a linear list of steps (either the root or a conditional branch).
struct BranchState {
  SequenceStep *steps;     // Pointer to an array of steps in this branch.
  int count;               // Total number of steps in the branch.
  int index;               // Index of the next step to execute.
  unsigned long branchStartTime;  // The micros() time when this branch started.
};
#define MAX_BRANCH_DEPTH 10
BranchState branchStack[MAX_BRANCH_DEPTH];
int branchStackTop = -1;
bool sequenceExecuting = false;

// A simple evaluator for conditions. It supports conditions such as "ARMING>3" and "BATT<4".
bool evaluateCondition(const String &cond) {
  if (cond.startsWith("ARMING>")) {
    float threshold = cond.substring(7).toFloat();
    int adc = analogRead(ARMING_SENSE_PIN);
    float voltage = adc / 197.0; // Conversion factor from telemetry.
    return voltage > threshold;
  } else if (cond.startsWith("BATT<")) {
    float threshold = cond.substring(5).toFloat();
    int adc = analogRead(BATT_SENSE_PIN);
    float voltage = adc / 28.5;  // Conversion factor.
    return voltage < threshold;
  }
  return false;
}

// startSequenceExecution() initializes the branch stack with the root sequence.
void startSequenceExecution() {
  branchStackTop = 0;
  branchStack[0].steps = rootSequence;
  branchStack[0].count = rootSequenceCount;
  branchStack[0].index = 0;
  branchStack[0].branchStartTime = micros();
  sequenceExecuting = true;
}

// processSequence() is called repeatedly in the loop() to advance execution.
void processSequence() {
  if (!sequenceExecuting) return;
  unsigned long now = micros();
  if (branchStackTop < 0) {
    sequenceExecuting = false;
    return;
  }
  BranchState *current = &branchStack[branchStackTop];
  if (current->index >= current->count) {
    // Finished current branch; pop the stack.
    branchStackTop--;
    return;
  }
  SequenceStep &step = current->steps[current->index];
  unsigned long scheduledTime = current->branchStartTime + step.timeOffset;
  if (now < scheduledTime) return; // Wait until the scheduled time arrives.
  
  // Process the step based on its type.
  if (step.type == STEP_EVENT) {
    // Execute the event: set the solenoid state.
    digitalWrite(solenoidPins[step.channel - 1], step.state);
    solenoidStates[step.channel - 1] = (step.state == HIGH);
    current->index++;
  } else if (step.type == STEP_DELAY) {
    // For a delay, simply advance the index.
    current->index++;
  } else if (step.type == STEP_CONDITIONAL) {
    // For a conditional, check if the condition is met.
    if (evaluateCondition(step.condition)) {
      // Condition met: push the true branch onto the stack.
      if (step.trueBranchCount > 0 && branchStackTop < MAX_BRANCH_DEPTH - 1) {
        branchStackTop++;
        branchStack[branchStackTop].steps = step.trueBranch;
        branchStack[branchStackTop].count = step.trueBranchCount;
        branchStack[branchStackTop].index = 0;
        branchStack[branchStackTop].branchStartTime = now;
      }
      current->index++;
    } else {
      // Condition not yet met; check if the timeout has occurred.
      if (now >= (scheduledTime + step.timeout)) {
        // Timeout reached: push the false branch.
        if (step.falseBranchCount > 0 && branchStackTop < MAX_BRANCH_DEPTH - 1) {
          branchStackTop++;
          branchStack[branchStackTop].steps = step.falseBranch;
          branchStack[branchStackTop].count = step.falseBranchCount;
          branchStack[branchStackTop].index = 0;
          branchStack[branchStackTop].branchStartTime = now;
        }
        current->index++;
      }
      // Otherwise, wait for the condition or timeout.
    }
  }
}

// ----- Telemetry Function -----
//
// buildTelemetry() creates a status string that includes sensor readings and solenoid states.
void buildTelemetry() {
  unsigned long t = millis();
  int adcBattery = analogRead(BATT_SENSE_PIN);
  float batteryVoltage = adcBattery / 28.5;
  int adcArming = analogRead(ARMING_SENSE_PIN);
  float armingVoltage = adcArming / 197.0;
  
  int n = snprintf(telemetryBuffer, TELEMETRY_BUFFER_SIZE,
                   "TS:%lu | ARM:%d | BATT:%.2fV | ARM_SENSE:%.2fV | SOL:",
                   t, digitalRead(ARM_PIN), batteryVoltage, armingVoltage);
  for (int i = 0; i < numSolenoids; i++) {
    int written = snprintf(telemetryBuffer + n, TELEMETRY_BUFFER_SIZE - n,
                             "%d:%s%s", i+1, solenoidStates[i] ? "ON" : "OFF",
                             (i < numSolenoids - 1 ? "," : ""));
    n += written;
    if(n >= TELEMETRY_BUFFER_SIZE) break;
  }
  int len = snprintf(telemetryBuffer + n, TELEMETRY_BUFFER_SIZE - n, " | SEQ:%s\n",
                     (sequenceExecuting ? "RUNNING" : "none"));
  n += len;
  telemetryLength = n;
  telemetryIndex = 0;
}

// ----- Command Processing (Interpreter) -----
//
// The processCommand() function is the entry point for command strings. It handles:
//   - "command:" commands to parse and store a new sequence,
//   - "sequence" to start execution,
//   - "abort" to cancel the current sequence,
//   - and direct solenoid commands.
void processCommand(const String &cmd) {
  String command = cmd;
  command.trim();
  if (command.length() == 0)
    return;
  
  // "abort" command: free memory and cancel execution.
  if (command.equalsIgnoreCase("abort")) {
    freeSequenceSteps(rootSequence, rootSequenceCount);
    rootSequenceCount = 0;
    sequenceExecuting = false;
    return;
  }
  
  // "sequence" command: start execution of the stored sequence.
  if (command.equalsIgnoreCase("sequence")) {
    if (rootSequenceCount > 0)
      startSequenceExecution();
    return;
  }
  
  // "command:" command: parse a new sequence.
  if (command.startsWith("command:")) {
    // Free previously allocated memory.
    freeSequenceSteps(rootSequence, rootSequenceCount);
    rootSequenceCount = 0;
    
    String seqStr = command.substring(8);
    int pos = 0;
    // Parse the root sequence using '.' as the separator.
    rootSequenceCount = parseSequence(seqStr, pos, rootSequence, MAX_ROOT_STEPS, '.');
    
    // Debug: print the parsed sequence tree to Serial.
    Serial.println("Parsed Sequence Tree:");
    for (int i = 0; i < rootSequenceCount; i++) {
      Serial.print(i); Serial.print(": ");
      if (rootSequence[i].type == STEP_EVENT) {
        Serial.print("EVENT s"); Serial.print(rootSequence[i].channel);
        Serial.print(rootSequence[i].state == HIGH ? "1" : "0");
        Serial.print(" @"); Serial.print(rootSequence[i].timeOffset); Serial.println("us");
      } else if (rootSequence[i].type == STEP_DELAY) {
        Serial.print("DELAY "); Serial.print(rootSequence[i].timeOffset); Serial.println("us");
      } else if (rootSequence[i].type == STEP_CONDITIONAL) {
        Serial.print("IF("); Serial.print(rootSequence[i].condition);
        Serial.print(") timeout "); Serial.print(rootSequence[i].timeout); Serial.println("us");
        Serial.print("  True branch count: "); Serial.println(rootSequence[i].trueBranchCount);
        Serial.print("  False branch count: "); Serial.println(rootSequence[i].falseBranchCount);
      }
    }
    return;
  }
  
  // Otherwise, allow direct solenoid commands (e.g., "s11").
  if (tolower(command.charAt(0)) == 's') {
    char stateChar = command.charAt(command.length() - 1);
    int stateVal = (stateChar == '1') ? HIGH : LOW;
    String channelStr = command.substring(1, command.length() - 1);
    int channel = channelStr.toInt();
    if (channel >= 1 && channel <= numSolenoids) {
      digitalWrite(solenoidPins[channel - 1], stateVal);
      solenoidStates[channel - 1] = (stateVal == HIGH);
    }
    return;
  }
}

//
// ----- Setup & Loop -----
//
void setup() {
  Serial.begin(2000000);
  Serial3.begin(2000000);
  
  // Initialize arm/disarm pins.
  pinMode(ARM_PIN, OUTPUT);
  pinMode(DISARM_PIN, OUTPUT);
  digitalWrite(ARM_PIN, LOW);
  digitalWrite(DISARM_PIN, HIGH);
  
  // Initialize solenoid pins.
  for (int i = 0; i < numSolenoids; i++) {
    pinMode(solenoidPins[i], OUTPUT);
    digitalWrite(solenoidPins[i], LOW);
    solenoidStates[i] = false;
  }
  
  // Initialize sensor pins.
  pinMode(BATT_SENSE_PIN, INPUT);
  pinMode(ARMING_SENSE_PIN, INPUT);
  pinMode(41, OUTPUT);
  digitalWrite(41, HIGH);
  
  buildTelemetry();
}

void loop() {
  // Nonblocking command reception from Serial3.
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
  
  // Nonblocking telemetry transmission.
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
  
  // Process sequence execution.
  if (sequenceExecuting) {
    processSequence();
  }
}
