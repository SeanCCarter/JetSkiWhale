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
#include <math.h>

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
int M1 = 4; 
int leftMotor = 6;                      
int M2 = 7; 

//Assign SharpIR sensors
SharpIR rightSensor(GP2YA41SK0F, rightIrPin);
SharpIR leftSensor(GP2YA41SK0F, leftIrPin);

//Battery Voltage Tracker
int measurementArray[10]={5,5,5,5,5,5,5,5,5,5}; //junk data
int voltage = 5; //voltage

////Assign Motors
//AF_DCMotor rightMotor(rightMotorPort);
//AF_DCMotor leftMotor(leftMotorPort);

//Assign constants for logic
int MINDISTANCE = 20;


void setup() {
  pinMode(ledPin, OUTPUT); //Allow output to pin with LED
  pinMode(rightIrPin, INPUT);
  pinMode(leftIrPin, INPUT);
  pinMode(batteryPin, INPUT);
  pinMode(M1, OUTPUT);   
  pinMode(M2, OUTPUT); 
  digitalWrite(M1,LOW);   
  digitalWrite(M2, LOW);  
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
//      rightMotor.setSpeed(255);
//      leftMotor.setSpeed(0);
        analogWrite(rightMotor, 255);
        analogWrite(leftMotor, 0);
    }
    if (rightDistance-leftDistance >= 0){
//      rightMotor.setSpeed(0);
//      leftMotor.setSpeed(255);
        analogWrite(rightMotor, 0);
        analogWrite(leftMotor, 255);
    }
  }
  else {
//    rightMotor.setSpeed(255);
//    leftMotor.setSpeed(255);
      analogWrite(rightMotor, 255);
      analogWrite(leftMotor, 255);
  }
  //Check battery voltage (TODO)
   
  /* Act:
  *  Set outputs
  */
  blinkSet(200); //blink every 200 millis
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
 * Function: checkBattery 
 *  Relies on: measurementArray global variable
 *  Returns: battery voltage (average of last 10 values)
 */
int checkBattery(){
  static int measurementArray[10] = {5,5,5,5,5,5,5,5,5,5};
  static int index = 0;
  int voltage = 0;

  meassurementArray[index] = analogRead(A2); //replace oldest voltage sample
  index = (index+1)%10; //make sure index is within size of the array

  for (int i=0; i<10; i++){
    voltage += meassurementArray[i] //sum up voltages
  }
  voltage = voltage/10; //divide by number of samples
  return voltage;
}




