#include <Stepper.h> // Include the Stepper library
#include "Arduino.h"
#include "bipolarstepper.h"

// DC Motor Pin Definitions
#define IN1_A 12 // Motor A IN1 pin
#define IN2_A 14 // Motor A IN2 pin
#define ENABLE_A 13 // Motor A ENA pin

#define IN1_B 27 // Motor B IN1 pin
#define IN2_B 26 // Motor B ENA pin
#define ENABLE_B 25 // Motor B ENA pin

int led = 2;
bool dirCW = false;
bool dirCCW = true;

// Stepper Motor Pin Definitions
BiPolStepper stepperA(15, 2, 32, 33, 2048);
BiPolStepper stepperB(5, 18, 19, 21, 2048);

const int FULL_SPEED = 255; // Maximum duty cycle for full speed
bool stepperActive = true;

void setup() {
  Serial.begin(9600); 

  // Set DC motor control pins as outputs
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  pinMode(ENABLE_A, OUTPUT);

  pinMode(IN1_B, OUTPUT);
  pinMode(IN2_B, OUTPUT);
  pinMode(ENABLE_B, OUTPUT);

  // Set initial stepper motor rotation directions
  stepperA.rotateFull(10, dirCW);
  stepperB.rotateFull(10, dirCCW);

  Serial.println("Motor control ready. Send 'MAJU' to run DC Motor A & B, 'STOP' to stop DC Motor A & B.");
  Serial.println("Send 'KIRI' to run DC Motor A, 'KANAN' to run DC Motor B.");
  Serial.println("Send 'STEPPER' to toggle Stepper Motor ON/OFF.");
}

void loop() {
  // Check for incoming commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\r\n');
    IN1_A = IN1
    IN2_A = IN2
    IN1_B = IN3
    IN2_B = IN4
    
    if (command == "MAJU") {
      digitalWrite(IN1_A, HIGH);
      digitalWrite(IN2_A, LOW);
      analogWrite(ENABLE_A, FULL_SPEED); // Full speed for Motor A
      digitalWrite(IN1_B, LOW);
      digitalWrite(IN2_B, HIGH);
      analogWrite(ENABLE_B, FULL_SPEED); // Full speed for Motor B
      Serial.println("DC Motor A and B running.");
    }

    if (command == "KANAN") {
      digitalWrite(IN1_A, LOW);
      digitalWrite(IN2_A, LOW);
      analogWrite(ENABLE_A, 0); // Stop Motor A
      digitalWrite(IN1_B, LOW);
      digitalWrite(IN2_B, HIGH);
      analogWrite(ENABLE_B, FULL_SPEED); // Full speed for Motor B
      Serial.println("DC Motor B running.");
    }

    if (command == "KIRI") {
      digitalWrite(IN1_A, HIGH);
      digitalWrite(IN2_A, LOW);
      analogWrite(ENABLE_A, FULL_SPEED); // Full speed for Motor A
      digitalWrite(IN1_B, LOW);
      digitalWrite(IN2_B, LOW);
      analogWrite(ENABLE_B, 0); // Stop Motor B
      Serial.println("DC Motor A running.");
    }
    
    if (command == "STOP") {
      digitalWrite(IN1_A, LOW);
      digitalWrite(IN2_A, LOW);
      analogWrite(ENABLE_A, 0); // Stop Motor A
      digitalWrite(IN1_B, LOW);
      digitalWrite(IN2_B, LOW);
      analogWrite(ENABLE_B, 0); // Stop Motor B
      Serial.println("DC Motor A and B stopped.");
    }
    
    if (command == "STEPPER") { 
      stepperActive = !stepperActive; // Toggle the stepper state
      Serial.println(stepperActive ? "Stepper Turned ON" : "Stepper Turned OFF");
    }

    else {
      Serial.println("Invalid command.");
      Serial.println("Check setting -> send -> new line : LF");
    }
  }

  // Continuously update stepper motors if active
  if (stepperActive) {
    stepperA.update();
    stepperB.update();
  }
}
