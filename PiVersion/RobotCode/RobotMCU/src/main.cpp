/*
 * File: robotMCU
 * --------------
 * Erick Blankenberg, Beck Goodloe, Josiah Clark
 * ME210
 * 2/5/2019
 * Final
 *
 * Description:
 * 	Allows for the raspberry pi to drive around
 */

/*------------------------------ Library Includes ----------------------------*/

#include <Arduino.h>
#include <Metro.h>

/*------------------------------ Pin Definitions -----------------------------*/

// Utility
#define PIN_LED 13

// Bumper pins, make sure to use input pullup. Is high when depressed.
#define PIN_RIGHT_BUMPER  8
#define PIN_LEFT_BUMPER  5
#define PIN_BACK_BUMPER 6
#define PIN_FRONT_BUMPER 7 // BAD

// Motor pins
#define PIN_NORTH_MOTOR_1 4
#define PIN_NORTH_MOTOR_2 3
#define PIN_SOUTH_MOTOR_1 10
#define PIN_SOUTH_MOTOR_2 9
#define PIN_EAST_MOTOR_1  17
#define PIN_EAST_MOTOR_2  16
#define PIN_WEST_MOTOR_1  22
#define PIN_WEST_MOTOR_2  23

/*----------------------------- Global Variables -----------------------------*/

// Timeout timer
#define TIMEOUT 50 // (ms) how long until bot should halt if connection fails
static Metro TimeOut = Metro(TIMEOUT);

/*----------------------------------- Main -----------------------------------*/

void setup() {
  // Really runs at full USB speed
  Serial.begin(9600);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  //Drive train pins
  pinMode(PIN_NORTH_MOTOR_1, OUTPUT);
  pinMode(PIN_NORTH_MOTOR_2, OUTPUT);
  pinMode(PIN_SOUTH_MOTOR_1, OUTPUT);
  pinMode(PIN_SOUTH_MOTOR_2, OUTPUT);
  pinMode(PIN_EAST_MOTOR_1, OUTPUT);
  pinMode(PIN_EAST_MOTOR_2, OUTPUT);
  pinMode(PIN_WEST_MOTOR_1, OUTPUT);
  pinMode(PIN_WEST_MOTOR_2, OUTPUT);

  // Bumper pins
  pinMode(PIN_BACK_BUMPER, INPUT_PULLUP);
  pinMode(PIN_FRONT_BUMPER, INPUT_PULLUP);
  pinMode(PIN_LEFT_BUMPER, INPUT_PULLUP);
  pinMode(PIN_RIGHT_BUMPER, INPUT_PULLUP);

}

void loop() {
  int currentState = 0; // Packed data we want to send, current format is UDLR----.....
  /*
  //Actuates moving forward
  analogWrite(PIN_NORTH_MOTOR_1, 1024);
  analogWrite(PIN_NORTH_MOTOR_2, 0);
  analogWrite(PIN_SOUTH_MOTOR_1, 1024);
  analogWrite(PIN_SOUTH_MOTOR_2, 0);
  analogWrite(PIN_EAST_MOTOR_1, 1024);
  analogWrite(PIN_EAST_MOTOR_2, 0);
  analogWrite(PIN_WEST_MOTOR_1, 1024);
  analogWrite(PIN_WEST_MOTOR_2, 0);
  while(1);
  */
  // Reads bumpers TODO debounce bumpers
  currentState |= ((int) digitalRead(PIN_FRONT_BUMPER)) << 0;
  currentState |= ((int) digitalRead(PIN_BACK_BUMPER))  << 1;
  currentState |= ((int) digitalRead(PIN_LEFT_BUMPER))  << 2;
  currentState |= ((int) digitalRead(PIN_RIGHT_BUMPER)) << 3;

  // Reads command input
  static int16_t motorValUpDown;
  static int16_t motorValLeftRight;
  if(TimeOut.check()) {

  } // Timeout

  // Updates motor values
  if(Serial.available()) {
    char commandHeader = Serial.read();
    switch(commandHeader) {
      case 'P': // Ping the MCU
        Serial.println("Here!");
        break;
      case 'U': // Update motor values
        TimeOut.reset();
        motorValUpDown    = Serial.parseInt();
        motorValLeftRight = Serial.parseInt();
        break;
      case 'B': // Get bumper values
        Serial.print(currentState);
        break;
      default:
        Serial.print("Parse Failure: \"");
        Serial.print(commandHeader);
        Serial.print("\"");
        digitalWrite(PIN_LED, HIGH); // Failure
    }
  }
  while(Serial.available()) {Serial.read();} // Clears up, faster than sending so should be ok

  // Actuates motors
  // > North/South
  analogWrite(PIN_NORTH_MOTOR_2, (motorValUpDown >= 0) ? motorValUpDown : 0);
  analogWrite(PIN_NORTH_MOTOR_1, (motorValUpDown < 0) ? abs(motorValUpDown) : 0);
  analogWrite(PIN_SOUTH_MOTOR_2, (motorValUpDown >= 0) ? motorValUpDown : 0);
  analogWrite(PIN_SOUTH_MOTOR_1, (motorValUpDown < 0) ? abs(motorValUpDown) : 0);
  // > East/West
  analogWrite(PIN_EAST_MOTOR_2, (motorValLeftRight >= 0) ? motorValLeftRight : 0);
  analogWrite(PIN_EAST_MOTOR_1, (motorValLeftRight < 0) ? abs(motorValLeftRight) : 0);
  analogWrite(PIN_WEST_MOTOR_2, (motorValLeftRight >= 0) ? motorValLeftRight : 0);
  analogWrite(PIN_WEST_MOTOR_1, (motorValLeftRight < 0) ? abs(motorValLeftRight) : 0);
}
