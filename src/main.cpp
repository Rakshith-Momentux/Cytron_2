#include <Arduino.h>

const int pinPwm = 9;    // PWM is connected to pin 9.
const int pinDir = 8;    // DIR is connected to pin 8.
static int iSpeed = 50;  // Speed of the motor.

volatile long encoderValue = 0;
const int encoderPinA = 2;  // Attach to interrupt pin
const int encoderPinB = 3;  // Attach to interrupt pin

unsigned long startTime; // Variable to store the start time
unsigned long runTime = 10000;     // Run time in milliseconds (10 seconds)

int gear_ratio_motor = 174;
int PPR_motor = 13;

long final_pos = 9048;

void readEncoder();

void setup() 
{    
  Serial.begin(115200);
  Serial.print("Changed ");
  // Initialize the PWM and DIR pins as digital outputs.
  pinMode(pinPwm, OUTPUT);
  pinMode(pinDir, OUTPUT);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), readEncoder, RISING);

  startTime = millis();  // Capture the start time
}

void loop() 
{
  // Check if 10 seconds have passed
  if (final_pos - encoderValue >= 10) 
  {
    analogWrite(pinPwm, iSpeed);
    digitalWrite(pinDir, HIGH);

    //float rotations = encoderValue / 13.0;
    Serial.print("Position is: ");
    Serial.println(encoderValue);

    delay(10);
  } 
  else 
  {
    analogWrite(pinPwm, 0);  // Stop the motor
    Serial.print("Final position is :");
    Serial.println(encoderValue);
    int last_val = encoderValue;
    float rotations = last_val / float(gear_ratio_motor * PPR_motor);
    Serial.print("Number of rotations is :");
    Serial.println(rotations,4);
    Serial.println("Motor stopped.");
    while (1);  // Stop the loop from running further
  }
}

void readEncoder() 
{
  // Read the current state of ENCA and ENCB
  bool encAState = digitalRead(encoderPinA);
  bool encBState = digitalRead(encoderPinB);
  // Serial.print(">Encoder A: ");
  // Serial.println(encAState);
  // Serial.print(">Encoder B: ");
  // Serial.println(encBState);

  if (encAState > 0) 
  {
    encoderValue++;
  } 
  else 
  {
    encoderValue--;
  }
}
