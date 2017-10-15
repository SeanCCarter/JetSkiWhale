/** 
 *  Program Name: whaledo
 *  Purpose: Causes whale-robot to wander randomly in pool
 *  Warnings: Do not suddenly unplug anything, it will work poorly
 *  Authors: Team Whaledo
 *  Edited: 9/22/2017
**/
//import libraries
#include <SharpIR.h>

/* Set up Global variables
*/

//Set up pins and outputs
int ledPin = 8;
int rightIrPin = A0;
int leftIrPin = A1;
int batteryPin = A2;
int rightMotorPort = 1;
int leftMotorPort = 2;

//Arduino PWM Speed Control
int rightMotor = 5;  
int rightDir = 4; //right motor direction
int leftMotor = 6;                      
int leftDir = 7; //left motor direction

//Assign SharpIR sensors
SharpIR rightSensor(GP2YA41SK0F, rightIrPin);
SharpIR leftSensor(GP2YA41SK0F, leftIrPin);

//Battery Voltage Tracker
int batteryVoltage;
int blinkPeriod = 200; //Blink light every {x} miliseconds

//Assign constants for logic
int MINDISTANCE = 15;

void setup() {
  //Set pins to input/output
  pinMode(ledPin, OUTPUT); //Allow output to pin with LED
  pinMode(rightIrPin, INPUT);
  pinMode(leftIrPin, INPUT);
  pinMode(batteryPin, INPUT);
  pinMode(rightDir, OUTPUT);   
  pinMode(leftDir, OUTPUT); 
  digitalWrite(rightDir,LOW);  //pumps only drive one way 
  digitalWrite(leftDir, LOW); //pumps only drive one way 
  Serial.begin(9600); //Enable the serial comunication
}

void loop() {
  /* Sense:
  *  Check IR Distances
  *  Check battery voltage
  */
  int rightDistance = rightSensor.getDistance(); //Calculate the distance in centimeters
  int leftDistance = leftSensor.getDistance(); //Calculate the distance in centimeters
  printIrValues(leftDistance, rightDistance); //Print distances to serial
  
  /* Think:
  *  Decide which motors are on
  */
  if (rightDistance <= MINDISTANCE or leftDistance <= MINDISTANCE) {
    if (rightDistance-leftDistance < 0) { //turn left
        analogWrite(rightMotor, 255); //right motor on
        analogWrite(leftMotor, 0); //left off
        delay(750); //Stops robot from just following pool edge
    }
    if (rightDistance-leftDistance >= 0){ //turn right
        analogWrite(rightMotor, 0); //right motor off
        analogWrite(leftMotor, 255); //left on
        delay(750); //Stops robot from just following pool edge
    }
  }
  else { //No wall, go straight
      analogWrite(rightMotor, 255);
      analogWrite(leftMotor, 255);
  }
   
  /* Act:  
  *  Set LED
  */
  blinkSet(blinkPeriod); //blink every 200 millis
}

/* Function: blinkSet
 *  Relies on: ledPin global variable
 *  Input: period (time between blinks)
 *  Sets LED state as function of time passed
 */
void blinkSet(int period){
  if (millis()%period < period/2 ){
    digitalWrite(ledPin, LOW); //Turn LED off
  }
  else{
    digitalWrite(ledPin, HIGH); //Turn LED on
  }
}

/*
 * Function: printIrValues 
 *  Input: 
 *    - int leftDistance: reading from left IR sensor
 *    - int rightDistance: reading from right IR sensor
 *  Prints the IR sensor readings on 1 line, in human readable format
 */
void printIrValues(int leftDistance, int rightDistance){
  Serial.print("Left: ");
  Serial.print(leftDistance);
  Serial.print(", Right: ");
  Serial.print(rightDistance);
  Serial.print("\n");
}

/*
 * Function: checkBattery 
 *  Relies on: measurementArray global variable
 *  Returns: battery voltage (average of last 10 values)
 */
int checkBattery(){
  static int measurementArray[10] = {5,5,5,5,5,5,5,5,5,5};
  static int index = 0;
  int voltage = 0;

  measurementArray[index] = analogRead(A2); //replace oldest voltage sample
  index = (index+1)%10; //make sure index is within size of the array

  for (int i=0; i<10; i++){
    voltage += measurementArray[i]; //sum up voltages
  }
  voltage = voltage/10; //divide by number of samples
  return voltage;
}






