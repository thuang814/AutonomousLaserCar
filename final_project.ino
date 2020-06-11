/*
   -- New project --

   This source code of graphical user interface
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 2.4.3 or later version
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/
     - for ANDROID 4.3.1 or later version;
     - for iOS 1.3.5 or later version;

   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// RemoteXY select connection mode and include library
#define REMOTEXY_MODE__ESP32CORE_WIFI_POINT
#include <WiFi.h>

#include <RemoteXY.h>

// RemoteXY connection settings
#define REMOTEXY_WIFI_SSID "RemoteXY"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377


// RemoteXY configurate
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
{ 255, 26, 0, 0, 0, 51, 0, 8, 13, 0,
  5, 52, 8, 21, 30, 30, 2, 26, 31, 3,
  132, 29, 5, 33, 9, 2, 26, 1, 0, 51,
  26, 15, 15, 2, 31, 70, 105, 114, 101, 0,
  4, 160, 64, 44, 30, 9, 2, 26, 7, 5,
  64, 14, 28, 8, 2, 26, 2, 21
};

// this structure defines all the variables of your control interface
struct {

  // input variable
  int8_t joystick_1_x; // =-100..100 x-coordinate joystick position
  int8_t joystick_1_y; // =-100..100 y-coordinate joystick position
  uint8_t select_1; // =0 if select position A, =1 if position B, =2 if position C, ...
  uint8_t button_1; // =1 if button pressed, else =0
  int8_t slider_1; // =-100..100 slider position
  char edit_1[21];  // string UTF8 end zero

  // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)


/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#define PIN_BUTTON_1 13


#define LED 32
#define SERLAS 33

#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#include <ESP32Servo.h>
Servo serlas, vert;  // create servo object to control a servo

//define right motor control pins
#define right_motor_A 15
#define right_motor_B 14
#define right_motor_speed 27 //enable pin

//define left motor control pins
#define left_motor_A 13
#define left_motor_B 12
#define left_motor_speed 26 //enable pin

VL53L0X_RangingMeasurementData_t measure;


void fire() {
  digitalWrite(LED, HIGH);
  delay(20);
  digitalWrite(LED, LOW);
}

unsigned long timeShot = 0;
unsigned long timeBurst = 0;
unsigned long serlasTime = 0;


void shot() {

  int timeBetweenShots = random(50, 200);
  int horiStart = random(1, 180);
  int horiEnd = random(1, 180);
  int numShots = random(5, 20);

  int horiChange = (horiEnd - horiStart) / numShots;

    serlas.write(horiStart);
    
    delay(100);

    int shot = 0;

    while(shot < numShots){
      if(millis()>timeShot){
  
         serlas.write(horiStart);
     
         horiStart += horiChange;
      
         fire();

         timeShot = timeShot + timeBetweenShots;

         shot = shot + 1;
         
      }
    }
  
}

double pos = 0;
bool spin = false;
void detect() {
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    if (measure.RangeMilliMeter <= 300) {
      fire();
    }
  }
  if (spin) {
    pos = pos + 10;
    if (pos >= 180) {
      spin = false;
    }
  } else {
    pos = pos - 10;
    if (pos <= 0) {
      spin = true;
    }
  }
}

void stopMoving() {
  digitalWrite(right_motor_A, LOW);
  digitalWrite(right_motor_B, LOW);
  digitalWrite(left_motor_A, LOW);
  digitalWrite(left_motor_B, LOW);
}

void forward() {
  digitalWrite(right_motor_A, LOW);
  digitalWrite(right_motor_B, HIGH);
  digitalWrite(left_motor_A, HIGH);
  digitalWrite(left_motor_B, LOW);
  //    analogWrite(left_motor_speed, 255 * perY);
  //    analogWrite(right_motor_speed, 255 * perY);
}

void right() {
  digitalWrite(right_motor_A, LOW);
  digitalWrite(right_motor_B, LOW);
  digitalWrite(left_motor_A, HIGH);
  digitalWrite(left_motor_B, LOW);
  //    analogWrite(left_motor_speed, 255 * perX);
  //    analogWrite(right_motor_speed, (255 * perX) / 2);
}

void left() {
  digitalWrite(right_motor_A, LOW);
  digitalWrite(right_motor_B, HIGH);
  digitalWrite(left_motor_A, LOW);
  digitalWrite(left_motor_B, LOW);
  //    analogWrite(right_motor_speed, 255 * perX);
  //    analogWrite(left_motor_speed, (255 * perX) / 2);
}

void backwards() {
  digitalWrite(right_motor_A, HIGH);
  digitalWrite(right_motor_B, LOW);
  digitalWrite(left_motor_A, LOW);
  digitalWrite(left_motor_B, HIGH);
  //    analogWrite(left_motor_speed, 255 * perY);
  //    analogWrite(right_motor_speed, 255 * perY);
}

void Wheel (int y , int x) // v = motor speed, motor = pointer to an array of pins
{
  double perX = abs(x / 100.0);
  double perY = abs(y / 100.0);
  int negX = -1 * x;
  int test1 = perX * 255;
  int test2 = perX * 255;
  if (x == 0 && y == 0) {
    stopMoving();
  }
  //foward
  else if (y >= x && y >= negX) {
    forward();
  }
  //backwards
  else if (y <= x && y <= negX) {
    backwards();
  }
  //right
  else if (y < x && y > negX) {
    right();
  }
  //left
  else if (y > x && y < negX) {
    left();
  }
}

bool moveLeft = false;
bool moveForward = false;
bool moveRight = false;
bool moveBack = false;
int ranDirection = random(1, 2);
unsigned int moveTimeout = millis();

void autoDrive() {
  if (moveTimeout < millis() ) {
    ranDirection = random(1, 3);
    moveTimeout = millis() + 3000;
  }
  serlas.write(90);
  Serial.println(measure.RangeMilliMeter);
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    if (measure.RangeMilliMeter <= 500) {
      moveForward = false;
      if (ranDirection == 1) {
        moveRight = true;
      }
      if (ranDirection == 2) {
        moveLeft = true;
      }
    }
    if (measure.RangeMilliMeter > 500) {
      moveForward = true;
      moveRight = false;
      moveLeft = false;
    }
    if (measure.RangeMilliMeter < 100) {
      moveForward = false;
      moveRight = false;
      moveLeft = false;
      moveBack = true;
    }
    Serial.print("randirection ");
    Serial.println(ranDirection);
  }
  delay(100);
  if (moveForward) {
    forward();
    Serial.println("foward...");
  }
  else if (moveRight) {
    right();
    Serial.println("right...");
  }
  else if (moveLeft) {
    left();
    Serial.println("left...");
  }
  else if (moveBack) {
    backwards();
    Serial.println("back...");
  } else {
    stopMoving();
    Serial.println("im not moving");
  }
}

void setup()
{
  Serial.begin(9600);

  RemoteXY_Init ();


  // TODO you setup code

  pinMode(LED, OUTPUT);
  serlas.attach(SERLAS);
  RemoteXY.slider_1 = 0;
  pinMode (right_motor_A, OUTPUT);
  pinMode (right_motor_B, OUTPUT);
  pinMode (left_motor_A, OUTPUT);
  pinMode (left_motor_B, OUTPUT);
  pinMode (right_motor_speed, OUTPUT);
  pinMode (left_motor_speed, OUTPUT);
  lox.begin();
}

void loop()
{

    RemoteXY_Handler ();
  
  
  
  
    strcpy(RemoteXY.edit_1, "Nothing");
  
    double serDegree = RemoteXY.slider_1 * 0.9;
  
    if (RemoteXY.select_1 == 0) {
      serlas.write(90 - serDegree);
    }
  
    if (RemoteXY.select_1 == 2) {
      serlas.write(pos);
      detect();
    } else {
      pos = 0;
    }
  
    lox.rangingTest(&measure, false);  //pass in 'true' to get debug data printout!
    if (measure.RangeStatus != 4) {   //phase failures have incorrect data
      if (measure.RangeMilliMeter <= 300) {
        strcpy(RemoteXY.edit_1, "ALERT!");
      }
    }
  
    digitalWrite(LED, (RemoteXY.button_1 == 0) ? LOW : HIGH);
  
    if (RemoteXY.select_1 == 1) {
      int timeBetweenBursts = random(200, 1000);
  
      if (millis() > timeBurst) {
        shot();
        timeBurst = millis() + timeBetweenBursts;
      }
  
    }
  
    if (RemoteXY.select_1 == 3) {
      autoDrive();
    } else {
      Wheel(RemoteXY.joystick_1_y , RemoteXY.joystick_1_x);
    }
}
