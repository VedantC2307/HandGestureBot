#include <Arduino.h>

const int numberOfMotors = 5;
const int stepPins[numberOfMotors] = {4, 6, 5, 8, 7}; // STEP pins for each motor
const int dirPins[numberOfMotors] = {9, 11, 10, 13, 12}; // DIR pins for each motor
int currentPositions[numberOfMotors] = {0, 0, 0, 0, 0}; // Track current position of each motor
unsigned long lastStepTime[numberOfMotors] = {0, 0, 0, 0, 0}; // Track last step time for each motor
int stepsRemaining[numberOfMotors] = {0, 0, 0, 0, 0}; // Steps remaining for current movement
const unsigned long stepInterval = 1000; // Microseconds between steps

String incomingString = ""; // To hold the incoming serial data

void setup() {
  Serial.begin(9600);
  // Initialize all the motor pins as outputs
  for (int i = 0; i < numberOfMotors; i++) {
    pinMode(stepPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
  }
  Serial.println("Ready to receive step counts for motors, separated by commas.");
}

void loop() {
  if (Serial.available() > 0) {
    incomingString = Serial.readStringUntil('\n');
    processIncomingString(incomingString);
  }
  for (int i = 0; i < numberOfMotors; i++) {
    if (stepsRemaining[i] != 0) {
      unsigned long currentMillis = micros();
      if (currentMillis - lastStepTime[i] >= stepInterval) {
        lastStepTime[i] = currentMillis;
        stepMotor(i);
      }
    }
  }
}

void processIncomingString(String data) {
  int motorIndex = 0;
  int fromIndex = 0;
  while (motorIndex < numberOfMotors && fromIndex != -1) {
    int endIndex = data.indexOf(',', fromIndex);
    if (endIndex == -1) { // If it's the last number in the string
      endIndex = data.length();
    }
    int targetPosition = data.substring(fromIndex, endIndex).toInt();
    stepsRemaining[motorIndex] = targetPosition - currentPositions[motorIndex];
    digitalWrite(dirPins[motorIndex], stepsRemaining[motorIndex] >= 0 ? HIGH : LOW);
    stepsRemaining[motorIndex] = abs(stepsRemaining[motorIndex]);
    currentPositions[motorIndex] = targetPosition;
    fromIndex = endIndex + 1;
    motorIndex++;
  }
}

void stepMotor(int motorIndex) {
  if (stepsRemaining[motorIndex] > 0) {
    digitalWrite(stepPins[motorIndex], HIGH);
    delayMicroseconds(100); // Pulse duration
    digitalWrite(stepPins[motorIndex], LOW);
    stepsRemaining[motorIndex]--;
  }
}
