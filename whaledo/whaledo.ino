/** 
 *  Program Name: whaledo
 *  Purpose: Causes whale-robot to wander randomly in pool
 *  Warnings: Do not suddenly unplug anything, it will work poorly
 *  Authors: Team Whaledo
 *  Edited: 9/22/2017
**/
//import libraries
#include <SharpIR.h>
#include <AFMotor.h>


/* Set up Global variables
*/

//Set up pins and outputs
int ledPin = 7;
int rightIrPin = A0;
int leftIrPin = A1;
int batteryPin = A2;
int rightMotorPort = 1;
int leftMotorPort = 2;


//Assign SharpIR sensors
SharpIR rightSensor(GP2YA41SK0F, rightIrPin);
SharpIR leftSensor(GP2YA41SK0F, leftIrPin);

//Assign Motors
AF_DCMotor rightMotor(rightMotorPort);
AF_DCMotor leftMotor(leftMotorPort);

//Assign constants for logic
int MINDISTANCE = 40;



void setup() {
  pinMode(ledPin, OUTPUT); //Allow output to pin with LED
  pinMode(rightIrPin, INPUT);
  pinMode(leftIrPin, INPUT);
  pinMode(batteryPin, INPUT);
  Serial.begin(9600); //Enable the serial comunication
}

void loop() {
  /* Sense:
  *  Check IR Distances
  *  Check battery voltage
  */
  int rightDistance = rightSensor.getDistance(); //Calculate the distance in centimeters
  int leftDistance = leftSensor.getDistance(); //Calculate the distance in centimeters
  Serial.print("Left: ");
  Serial.print(leftDistance);
  Serial.print(", Right: ");
  Serial.print(rightDistance);
  Serial.print("\n");
  //Check battery voltage (TODO)
  
  /* Think:
  *  Switch LED State
  *  Decide which motors are on
  */
  if (rightDistance <= MINDISTANCE or leftDistance <= MINDISTANCE) {
    if (rightDistance-leftDistance < 0) {
      rightMotor.setSpeed(255);
      leftMotor.setSpeed(0);
    }
    if (rightDistance-leftDistance >= 0){
      rightMotor.setSpeed(0);
      leftMotor.setSpeed(255);
    }
  }
  else {
    rightMotor.setSpeed(255);
    leftMotor.setSpeed(255);
  }
  //Check battery voltage (TODO)
   
  /* Act:
  *  Set outputs
  */
  blinkSet(400); //blink every 200 millis
}

/* Function: blinkSet
 *  Relies on: ledPin global variable
 *  Sets LED state based on time passed
 */
void blinkSet(int period){
  if (millis()%period < period/2 ){
    digitalWrite(ledPin, LOW); //Turn LED off
  }
  else{
    digitalWrite(ledPin, HIGH); //Turn LED on
  }
  
}


