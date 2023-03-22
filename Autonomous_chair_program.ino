#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <MechaQMC5883.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
// DC Motor 1 //
#define RPWM 46 //2 define pin 3 for RPWM pin (output)
#define R_EN 48 //3 define pin 2 for R_EN pin (input)
#define R_IS 50 //4 define pin 5 for R_IS pin (output)

#define LPWM 44 //5 define pin 6 for LPWM pin (output)
#define L_EN 42 //6 define pin 7 for L_EN pin (input)
#define L_IS 40 //7 define pin 8 for L_IS pin (output)
#define CW 1 //do not change
#define CCW 0 //do not change
#define debug 1 //change to 0 to hide serial monitor debugging infornmation or set to 1 to view
// DC Motor 2 //
#define RPWM_2 6 //  // define pin 3 for RPWM pin (output)
#define R_EN_2 45 //  // define pin 2 for R_EN pin (input)
#define R_IS_2 47 //  // define pin 5 for R_IS pin (output)

#define LPWM_2 5 //  // define pin 6 for LPWM pin (output)
#define L_EN_2 51 //  // define pin 7 for L_EN pin (input)
#define L_IS_2 49 //  // define pin 8 for L_IS pin (output)
#define CW_2 1 //do not change
#define CCW_2 0 //do not change
#define debug_2 1 //change to 0 to hide serial monitor debugging infornmation or set to 1 to view
//Step Motor //
#define DIR 2
#define STP 3
#define EN 4

// Chair Varriables
int speedChair = 50;   // speed in %
int currentPosition = 0;
int desiredPosition = 0;
String state;
char sliderValue;
int follow = 0;
int a;
float distanceToChair = 0;
int autonomousVariable = 0 ;
int minDistanceRight = 110;
int minDistanceLeft = 110;
int minDistanceFront = 120;
int minDistanceRotation = 90;
boolean distanceVisible = 0 ;
// Variables for median filtering
const int arraySize = 5;
int ultrasonicArray[arraySize][arraySize];
int currentValue[arraySize];

// Distance sensors echo and trig ( for 8 sensors) and variables declaration 
int echo [7];
int trig [7];
long h_time, h_distance;

// Including H Bridge BTS7960 library //
#include <RobojaxBTS7960.h>
RobojaxBTS7960 motor1(R_EN, RPWM, R_IS, L_EN, LPWM, L_IS, debug);
RobojaxBTS7960 motor2(R_EN_2, RPWM_2, R_IS_2, L_EN_2, LPWM_2, L_IS_2, debug_2);
// Including library need for HC06 Bluetooth module //

// Definitions 

#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif
MechaQMC5883 qmc;
float phoneLatitude;
float phoneLongtitude;
float chairLatitude;
float chairLongtitude;
int Bearing;
static const int RXPin = A10, TXPin = A11; /// do zmiany
static const uint32_t GPSBaud = 9600;
// The TinyGPS++ object
TinyGPSPlus gps;
// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
//
void setup() {

  Serial.begin(9600);// setup Serial Monitor to display information
  Serial1.begin(9600); //Initialize Bluetooth Serial Port

  // Starting DC motors //
  motor1.begin();
  motor2.begin();
  // Starting Step Motor //
  pinMode(DIR, OUTPUT);
  pinMode(STP, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);
 //Initializing HC -SR04 Ultrasonic distance sensors
  hcsr_begin(0, 11, 10); // (index,echo,trig)
  hcsr_begin(2, 13, 12); // (index,echo,trig)
  hcsr_begin(1, 8, 9);   // (index,echo,trig)
/// For GPS and compas///
  Wire.begin();
  qmc.init();
//  //qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);
  ss.begin(GPSBaud);

}

/////////Functions//////////////////////////////////////////
  void goAhead() {
    desiredPosition = 0;
    if (desiredPosition < currentPosition) {
    digitalWrite(EN, LOW);
    digitalWrite(DIR,HIGH); //Changes the rotations direction
     for (int x = 0; x < (currentPosition - desiredPosition); x++) {
    digitalWrite(STP,HIGH);
    delayMicroseconds(1000);
    digitalWrite(STP,LOW);
    delayMicroseconds(1000);
      }
    digitalWrite(EN, HIGH);      
      currentPosition = desiredPosition;
      delay(500);
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction 
    } else if (desiredPosition > currentPosition) {
      digitalWrite(EN, LOW);
      digitalWrite(DIR,LOW); //Changes the rotations direction
      for (int x = 0; x < (desiredPosition - currentPosition); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN, HIGH);
      delay(500);
      currentPosition = desiredPosition;
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction
    } else if (desiredPosition == currentPosition) {
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction
    }

  }
  void goBack(){
    desiredPosition = 0;
    if (desiredPosition < currentPosition) {
      digitalWrite(EN, LOW);
      digitalWrite(DIR,HIGH); //Changes the rotations direction      
      for (int x = 0; x < (currentPosition - desiredPosition); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN, HIGH);
      currentPosition = desiredPosition;
      delay(500);
      motor1.rotate(speedChair, CW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CCW_2); // run motor with selected by slider speed in CW direction
    
    } else if (desiredPosition > currentPosition) {
      digitalWrite(EN, LOW);
      digitalWrite(DIR,LOW); //Changes the rotations direction
      for (int x = 0; x < (desiredPosition - currentPosition); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN, HIGH);
      delay(500);
      currentPosition = desiredPosition;
      motor1.rotate(speedChair, CW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CCW_2); // run motor with selected by slider speed in CW direction
    } else if (desiredPosition == currentPosition) {
      motor1.rotate(speedChair, CW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CCW_2); // run motor with selected by slider speed in CW direction
    }
  }
  void goLeft(){
    desiredPosition = 60; // 50st = 30° 75st = 45° 
    if (desiredPosition < currentPosition) { //   Not possible in our case
      digitalWrite(EN, LOW);
      digitalWrite(DIR,HIGH); //Changes the rotations direction
      for (int x = 0; x < (currentPosition - desiredPosition); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN, HIGH);
      currentPosition = desiredPosition;
      delay(500);
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction 
    } else if (desiredPosition > currentPosition) {
      digitalWrite(EN, LOW);
      digitalWrite(DIR,LOW); //Changes the rotations direction
      for (int x = 0; x < (desiredPosition - currentPosition); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN, HIGH);
      delay(500);
      currentPosition = desiredPosition;
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction
    } else if (desiredPosition == currentPosition) {
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction
    }
  }
  void goRight(){
    desiredPosition = -60; // -50st = -30° -75st = -45° 
    if (desiredPosition < currentPosition) {
      digitalWrite(EN, LOW);
      digitalWrite(DIR,HIGH); //Changes the rotations direction
      for (int x = 0; x < (currentPosition - desiredPosition); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN, HIGH);
      currentPosition = desiredPosition;
      delay(500);
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction 
    /*} else if (desiredPosition > currentPosition) { // Not possible in our case
     * digitalWrite(DIR,HIGH); //Changes the rotations direction
      for (int x = 0; x < (abs(desiredPosition - currentPosition)); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(500);
      digitalWrite(STP,LOW);
      delayMicroseconds(500);
      }
      delay(500);
      currentPosition = desiredPosition;
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction */
    } else if (desiredPosition == currentPosition) {
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction
    }
  }
  void goAheadLeft(){
    desiredPosition = 50; // 25st = 15° 50st = 30°
    if (desiredPosition < currentPosition) { 
      digitalWrite(EN, LOW);
      digitalWrite(DIR,HIGH); //Changes the rotations direction
      for (int x = 0; x < (currentPosition - desiredPosition); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN, HIGH);
      currentPosition = desiredPosition;
      delay(500);
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction 
    } else if (desiredPosition > currentPosition) {
      digitalWrite(EN, LOW);
      digitalWrite(DIR,LOW); //Changes the rotations direction
      for (int x = 0; x < (desiredPosition - currentPosition); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN, HIGH);
      delay(500);
      currentPosition = desiredPosition;
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction
    } else if (desiredPosition == currentPosition) {
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction
    }
  }
  void goAheadRight(){
    desiredPosition = -50; // -25st = -15° -50st = -30°
    if (desiredPosition < currentPosition) { 
      digitalWrite(EN,LOW);
      digitalWrite(DIR,HIGH); //Changes the rotations direction      
      for (int x = 0; x < (currentPosition - desiredPosition); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN,HIGH);      
      currentPosition = desiredPosition;
      delay(500);
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction 
    } else if (desiredPosition > currentPosition) {
      digitalWrite(EN,LOW);
      digitalWrite(DIR,LOW); //Changes the rotations direction  
      for (int x = 0; x < (abs(desiredPosition - currentPosition)); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN,HIGH);  
      delay(500);
      currentPosition = desiredPosition;
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction
    } else if (desiredPosition == currentPosition) {
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction
    }
  }
  void goBackLeft(){
    desiredPosition = 50; // 25st = 15° 50st = 30°
    if (desiredPosition < currentPosition) {
      digitalWrite(EN, LOW);
      digitalWrite(DIR,HIGH); //Changes the rotations direction       
      for (int x = 0; x < (currentPosition - desiredPosition); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN, HIGH);
      currentPosition = desiredPosition;
      delay(500);
      motor1.rotate(speedChair, CW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CCW_2); // run motor with selected by slider speed in CW direction 
    } else if (desiredPosition > currentPosition) {
      digitalWrite(EN, LOW);
      digitalWrite(DIR,LOW); //Changes the rotations direction
      for (int x = 0; x < (desiredPosition - currentPosition); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN, HIGH);
      delay(500);
      currentPosition = desiredPosition;
      motor1.rotate(speedChair, CW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CCW_2); // run motor with selected by slider speed in CW direction
    } else if (desiredPosition == currentPosition) {
      motor1.rotate(speedChair, CW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CCW_2); // run motor with selected by slider speed in CW direction
    }
  }
  void goBackRight(){
    desiredPosition = -50; // -50st = -30° 
    if (desiredPosition < currentPosition) {
      digitalWrite(EN, LOW); 
      digitalWrite(DIR,HIGH); //Changes the rotations direction
      for (int x = 0; x < (currentPosition - desiredPosition); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN, HIGH);
      currentPosition = desiredPosition;
      delay(500);
      motor1.rotate(speedChair, CW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CCW_2); // run motor with selected by slider speed in CW direction 
    } else if (desiredPosition > currentPosition) {
      digitalWrite(EN, LOW);
      digitalWrite(DIR,LOW); //Changes the rotations direction
      for (int x = 0; x < (abs(desiredPosition - currentPosition)); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN, HIGH);
      delay(500);
      currentPosition = desiredPosition;
      motor1.rotate(speedChair, CW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CCW_2); // run motor with selected by slider speed in CW direction
    } else if (desiredPosition == currentPosition) {
      motor1.rotate(speedChair, CW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CCW_2); // run motor with selected by slider speed in CW direction
    }
  }
  void basePosition(){
    desiredPosition = 0;
    autonomousVariable = 0;
    follow = 0; 
    if (desiredPosition < currentPosition) {
      digitalWrite(EN, LOW);
      digitalWrite(DIR,HIGH); //Changes the rotations direction 
      for (int x = 0; x < (currentPosition - desiredPosition); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN, HIGH);
      currentPosition = desiredPosition;
      motor1.stop();// stop the motor
      motor2.stop();// stop the motor
    } else if (desiredPosition > currentPosition) {
      digitalWrite(EN, LOW);
      digitalWrite(DIR,LOW); //Changes the rotations direction
      for (int x = 0; x < (abs(desiredPosition - currentPosition)); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN, HIGH);
      currentPosition = desiredPosition;
      motor1.stop();// stop the motor
      motor2.stop();// stop the motor
    } else if (desiredPosition == currentPosition) {
      motor1.stop();// stop the motor
      motor2.stop();// stop the motor
    }
  }
  void stopChair(){
    motor1.stop();// stop the motor
    motor2.stop();// stop the motor
  }
  void goRotationRight(){
    int speedChair = 50;
    desiredPosition = -50; // -25st = -15° -50st = -30°
    
    if (desiredPosition < currentPosition) { 
      digitalWrite(EN,LOW);
      digitalWrite(DIR,HIGH); //Changes the rotations direction      
      for (int x = 0; x < (currentPosition - desiredPosition); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN,HIGH);      
      currentPosition = desiredPosition;
      delay(500);
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      //motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction 
    } else if (desiredPosition > currentPosition) {
      digitalWrite(EN,LOW);
      digitalWrite(DIR,LOW); //Changes the rotations direction  
      for (int x = 0; x < (abs(desiredPosition - currentPosition)); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN,HIGH);  
      delay(500);
      currentPosition = desiredPosition;
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      //motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction
    } else if (desiredPosition == currentPosition) {
      motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      //motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction
    }
  }  
  void goRotationLeft(){
    desiredPosition = 50; // 25st = 15° 50st = 30°
    int speedChair = 50;
    
    if (desiredPosition < currentPosition) { 
      digitalWrite(EN, LOW);
      digitalWrite(DIR,HIGH); //Changes the rotations direction
      for (int x = 0; x < (currentPosition - desiredPosition); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN, HIGH);
      currentPosition = desiredPosition;
      delay(500);
      //motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction 
    } else if (desiredPosition > currentPosition) {
      digitalWrite(EN, LOW);
      digitalWrite(DIR,LOW); //Changes the rotations direction
      for (int x = 0; x < (desiredPosition - currentPosition); x++) {
      digitalWrite(STP,HIGH);
      delayMicroseconds(1000);
      digitalWrite(STP,LOW);
      delayMicroseconds(1000);
      }
      digitalWrite(EN, HIGH);
      delay(500);
      currentPosition = desiredPosition;
      //motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction
    } else if (desiredPosition == currentPosition) {
      //motor1.rotate(speedChair, CCW); // run motor with selected by slider speed  in CCW direction
      motor2.rotate(speedChair, CW_2); // run motor with selected by slider speed in CW direction
    }
  }
  void executeAutonomousMode(){
    follow = 0;
    //autonomousVariable = 1;
    int speedChair = 45;
    
//    while(autonomousVariable = 1 ) {
    ////// Front //////  
      if (hcsr_getDistance(0) <= minDistanceFront) {
      //Serial.println("Too close");
        if ((hcsr_getDistance(1) <= hcsr_getDistance(2))& (hcsr_getDistance(2)>= minDistanceRight )& (hcsr_getDistance(0)> minDistanceRotation)){
        goRight();
      //Serial.println("Right");  
        }
        else if ((hcsr_getDistance(1) > hcsr_getDistance(2))& (hcsr_getDistance(0)> minDistanceRotation)& (hcsr_getDistance(1)>= minDistanceLeft)) {
        goLeft();
      //Serial.println("Left");
        }
        else {
        goBack();
        }  
      }
      else {
      goAhead();
      //Serial.println("Ahead"); 
      }
    ////////////////
   // }

  }
  void executeFollow(){
    autonomousVariable = 0;
    int speedChair = 50;
    if(a>= 180 && (a < Bearing < 360 || 0 < Bearing <(a-180)) && (abs(Bearing-a)) > 15){
      goRotationRight();
    }
    else if( a < 180 && (a<Bearing<(a+180)) && (abs(Bearing-a) > 15)) {
      goRotationRight();
    }
    else if (abs(Bearing-a) > 15) {
      goRotationLeft();
    }
    else if ((abs(Bearing - a) <= 15) && distanceToChair > 2.5){
      goAhead();
    }
    else stopChair();
  } 
//// GPS and compas functions /////
float geoBearing() {
  Serial.print(" chairlatitude: ");
  Serial.print (chairLatitude,7);
  Serial.print( '\t' );
  Serial.print(" chairLongtitude: ");
  Serial.print (chairLongtitude,7);
  Serial.print( '\t' );
  Serial.print(" phonelatitude: ");
  Serial.print (phoneLatitude,7);
  Serial.print( '\t' );
   Serial.print(" phoneLongtitude: ");
  Serial.print (phoneLongtitude,7);
  Serial.print( '\n' );
  
  float x = cos(phoneLatitude* DEGTORAD )* sin(phoneLongtitude * DEGTORAD - chairLongtitude * DEGTORAD );
//  Serial.print(" x: ");
//  Serial.print (x,7);
//  Serial.print( '\n' );  
  float y = cos(chairLatitude* DEGTORAD)*sin(phoneLatitude* DEGTORAD) - (sin(chairLatitude* DEGTORAD)*cos(phoneLatitude* DEGTORAD)*cos(phoneLongtitude* DEGTORAD - chairLongtitude * DEGTORAD));
//  Serial.print(" y: ");
//  Serial.print (y,7);
//  Serial.print('\n' ); 
  Bearing = atan2(x, y)* RADTODEG;
  Serial.print(" Bearing: ");
  Serial.print(Bearing);
  Serial.print('\n');
}
float geoDistance() {
//  Serial.print(" Dla distance: ");
//  Serial.print(" chairlatitude: ");
//  Serial.print (chairLatitude,7);
//  Serial.print( '\t' );
//  Serial.print(" chairLongtitude: ");
//  Serial.print (chairLongtitude,7);
//  Serial.print( '\t' );
//  Serial.print(" phonelatitude: ");
//  Serial.print (phoneLatitude,7);
//  Serial.print( '\t' );
//   Serial.print(" phoneLongtitude: ");
//  Serial.print (phoneLongtitude,7);
//  Serial.print( '\n' );
  const float R = 6371000 ;// m
  float p1 = chairLatitude * DEGTORAD;
//  Serial.print(" p1: ");
//  Serial.print (p1,7);
//  Serial.print( '\n' );
  float p2 = phoneLatitude * DEGTORAD;
  float dp = (chairLatitude-phoneLatitude) * DEGTORAD;
  float dl = (chairLongtitude-phoneLongtitude) * DEGTORAD;
  float p3 = chairLongtitude * DEGTORAD;
  float p4 = phoneLongtitude * DEGTORAD;//

  float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));
  distanceToChair = R *y ;

  /////
///  float distanceToChair = R* acos(sin(p1)*sin(p2)+ cos(p1)*cos(p2)*cos(p3-p4));
  ////
  Serial.print(" Distance to chair: ");  
  Serial.print(distanceToChair) ;
  Serial.print( '\n' );
}
//// GPS and compas functions end /////

////// HC_SR 04 Sensor Functions////////
  
//function hcsr_begin set new sensor
//index is the sensor serial number
//As declared in table in beginning there can be 8 sensors ( from number 0 to 7)
//hcsr_begin( 0, 11, 12); - means sensor nr 0, signal echo on pin 11,
//tigger signal on pin number 12

volatile void hcsr_begin(int index, int echoPin, int trigPin)
{


    echo[ index ] = echoPin;
    trig[ index ] = trigPin;

    pinMode( echo[ index ], INPUT);
    pinMode( trig[ index ], OUTPUT);
    
}

// function returns distance in cm
//as argument we use sensor serial number (from 0 to 7)
// hcsr_getDistance(0) get data from sensor 0; 
int hcsr_getDistance(int index) 
{

  digitalWrite(trig[ index], LOW);
  delayMicroseconds(2);
  digitalWrite(trig[ index], HIGH);
  delayMicroseconds(10);
  digitalWrite(trig[ index], LOW);

  h_time = pulseIn(echo[ index ], HIGH);
  if(h_time >= 23200) return 400;
  delay(50);
  return h_time / 58.00;
  delay(100);
}   
////// HC_SR 04 Sensor Functions End ////////

///////// Median filtering procedure /////////
// Swap two variables
int temp;
#define swap(w, z) temp = w; w = z; z = temp;
#define sort(x, y) if(x > y) { swap(x, y); }


// Median calculation
int ultrasonicMedian(int a, int b, int c, int d, int e) {
  sort(a, b);
  sort(d, e);
  sort(a, c);
  sort(b, c);
  sort(a, d);
  sort(c, d);
  sort(b, e);
  sort(b, c);
 //Serial.println(String("a=") + a + String(" b=") + b + String(" c=") + c + String(" d=") + d + String(" e=") + e);
 //Serial.print("\n");
  return c;
}

// keep track of last 5 values;
void rollingValues(int index) {
  for (int i = 4; i > 0; i--) {
    ultrasonicArray[index][0] = hcsr_getDistance(index);
    ultrasonicArray[index][i] = ultrasonicArray[index][i - 1];
  }
  // put the current value in the first position
 // ultrasonicArray[index][4] = currentValue[index];
}

int filtered_getDistance(int index){

  rollingValues(index);
  currentValue[index] = ultrasonicArray[index][4];
  

  // if this sensor value is different from the last value
  // by more than 20, replace with median
  if ((currentValue[index] > ultrasonicArray[index][0] + 20) ||(currentValue[index] < ultrasonicArray[index][0] - 20)) {
    currentValue[index] = ultrasonicMedian(
                            ultrasonicArray[index][0],
                            ultrasonicArray[index][1],
                            ultrasonicArray[index][2],
                            ultrasonicArray[index][3],
                            ultrasonicArray[index][4]
                          );
  //time for  CPU
  delayMicroseconds(50);                         
 return  currentValue[index];
  }
  else {
  //time for  CPU
  delayMicroseconds(50);
    return  currentValue[index];
  }
 

}

/////////  End of Median filtering procedure /////////

bool isNumber(String s) 
{ 
    for (int i = 0; i < s.length(); i++) 
        if (isdigit(s[i]) == false) 
            return false; 
  
    return true; 
} 
void displayInfo()
{
  Serial.print("Location: "); 
  if (gps.location.isValid())
  {
    Serial.print("Chair Latitude ");
    Serial.print(gps.location.lat(), 7);
    chairLatitude = gps.location.lat();
    Serial.print(",");
     Serial.print("Chair Longtitude ");
    Serial.print(gps.location.lng(), 7);
    chairLongtitude = gps.location.lng();
  }
  else
  {
    Serial.print("INVALID");
  }
  Serial.println();
}
void loop() {
    // put your main code here, to run repeatedly:
uint16_t x, y, z;
  int azimuth;
  //float azimuth; //is supporting float too
  qmc.read(&x, &y, &z,&azimuth);
  azimuth = qmc.azimuth(&y,&x);//you can get custom azimuth
  a = azimuth -37;
  if(a<0){
    a= a+360;
  }
  
//  azimuth = qmc.azimuth(&y,&x);//you can get custom azimuth
//  Serial.print("x: ");
//  Serial.print(x);
//  Serial.print(" y: ");
//  Serial.print(y);
//  Serial.print(" z: ");
//  Serial.print(z);
  Serial.print(" heading: ");
  Serial.print(a);
  Serial.println();
  delay(100);

//// GPS ///
// This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();
  //Read data from HC06
/*  if(Serial1.available()>0){
    state+=(char)Serial1.read();} */
   if(!Serial1.available()) state = ""; 
   while (Serial1.available()>0) {
    // may add small delay to allow input buffer to fill
    char c = Serial1.read();  //gets one byte from serial buffer
    state += c;
 //   delay(50);  
  } //makes the string readString 
  if(isNumber(state)){
    sliderValue = state[state.length() - 1];
  }
  //Serial.println("yyy");
  //Serial.println(state);
  if (state.startsWith("PHONEGPS"))
  {
    //Serial.println("xxx"+state);
    char cstr[state.length() + 1];
    strcpy(cstr, state.c_str());
    
    char *token = strtok(cstr, ";"); 
    for (int i=0;i<3;i++) 
    { 
      if(i!=0){
        // Serial.println(token); // in *char
        token = strtok(NULL, ";"); 
        if(i==1){
        phoneLatitude = atof(token); //in float
        Serial.print("Phone latitude ");
        Serial.println(phoneLatitude, 7);
        }
        if(i==2){
        phoneLongtitude = atof(token); //in float
        Serial.print("Phone longitude ");
        Serial.println(phoneLongtitude, 7);
        }
      }
    } 
  

  }
 delay(10);
  
if(state.length()>0) Serial.println("Whole string: "+ state); // Testing what information is send
  if (state == "F") goAhead();
  else if (state == "B") goBack();
  else if (state == "R") goRight();
  else if (state == "L") goLeft();
  else if (state == "FR") goAheadRight(); //FR
  else if (state == "FL") goAheadLeft(); //FL
  else if (state == "BR") goBackRight(); //BR
  else if (state == "BL") goBackLeft();  //BL
  else if (state == "RL") goRotationLeft();  // 
  else if (state == "RR") goRotationRight();  // 
  else if (sliderValue == '1') speedChair = 50 ; 
  else if (sliderValue == '2') speedChair = 60 ;
  else if (sliderValue == '3') speedChair = 70 ;
  else if (sliderValue == '4') speedChair = 80 ;
  else if (sliderValue == '5') speedChair = 90 ;
  else if (sliderValue == '6') speedChair = 100 ;
  else if (state == "BP") basePosition();
  else if (state == "S"||state == "FRS"||state == "FLS"||state == "BRS"||state == "BLS"||state == "FS"||state == "BS"||state == "RS"||state == "LS") stopChair();
  else if (state == "AUTONOMOUS") {
    autonomousVariable = 1 ;
    follow = 0;
    }
  else if (state == "MANUAL") {
    autonomousVariable = 0; 
    follow = 0;  
  }
  else if (state == "DISTANCE") distanceVisible = !distanceVisible; 
  else if (state == "follow"){
    autonomousVariable = 0;
    follow = 1; 
  }
  else if (state == "stopfollow"){
    follow = 0; 
  }
  else if(state!=""){ 
    stopChair();
    Serial.print("Command recieved : ");
    Serial.println(state); 
    }
  
//////// HC Sensor Commands ///////
delay(10);
 /*if(!hcsr_getDistance(0)) Serial.print("ERROR");
 else Serial.print(hcsr_getDistance(0));
 Serial.print("\t");*/
 if(distanceVisible == 1){
   // Send data to APP from Front Sensor
   if(!hcsr_getDistance(0)) Serial1.println("A;ERROR");
   else{
    //Serial.println(String("A;") + hcsr_getDistance(0));
    Serial1.println(String("A;") + hcsr_getDistance(0));
    delay(10);
   }
  
   
   /*if(!hcsr_getDistance(1)) Serial.print("ERROR");
   else Serial.print(hcsr_getDistance(1));
   Serial.print("\t"); */
   // Send data to APP from Left Sensor
   if(!hcsr_getDistance(1)) Serial1.println("B;ERROR");
   else {
    Serial1.println(String("B;") + hcsr_getDistance(1));
    //Serial.println(String("B;") + hcsr_getDistance(1));
    delay(10);
   }
   
   
   /*if(!hcsr_getDistance(2)) Serial.print("ERROR");
   else Serial.print(hcsr_getDistance(2));
   Serial.print("\n"); */
   // Send data to APP from Right Sensor
   if(!hcsr_getDistance(2)) Serial1.println("C;ERROR");
   else {
    Serial1.println(String("C;") + hcsr_getDistance(2));
    //Serial.println(String("C;") + hcsr_getDistance(2));
    delay(10);
  }
 }
 delay(10);
 if(autonomousVariable ==1)  {
    // Send data to APP from Front Sensor
   if(!hcsr_getDistance(0)) Serial1.println("A;ERROR");
   else{
    Serial.println(String("A;") + hcsr_getDistance(0));
    delay(10);
   }
   // Send data to APP from Left Sensor
   if(!hcsr_getDistance(1)) Serial1.println("B;ERROR");
   else {
    Serial.println(String("B;") + hcsr_getDistance(1));
    delay(10);
   }
  
   // Send data to APP from Right Sensor
   if(!hcsr_getDistance(2)) Serial1.println("C;ERROR");
   else {
    Serial.println(String("C;") + hcsr_getDistance(2));
    delay(10);
    }
 }
geoBearing();
geoDistance();
 
  delay(10);  
  if(autonomousVariable ==1)executeAutonomousMode();
  if(follow ==1)executeFollow();
  delay(25);     
} // loop ends
