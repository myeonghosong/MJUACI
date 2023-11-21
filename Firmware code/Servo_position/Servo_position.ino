/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <Dynamixel2Arduino.h>
//#include <SoftwareSerial.h>
#include <stdlib.h>
#include <math.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif
 

const uint8_t DXL_1 = 100;
const uint8_t DXL_2 = 75;
const uint8_t DXL_3 = 50;
const uint8_t DXL_4 = 25;

float f_present_position1 = 0.0;
float f_present_position2 = 0.0;
float f_present_position3 = 0.0;
float f_present_position4 = 0.0;

float servo = 35.25;
float short_servo = 14.25;
float long_joint = 55.0;
float short_joint = 28;

float link0 = servo;
float link1 = long_joint + servo;
float link2 = long_joint + servo;
float link3 = short_joint + short_servo;
float link4 = long_joint;
float y_min = 60;
int target = y_min;
int temp = 60;

const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_1);
  dxl.ping(DXL_2);
  dxl.ping(DXL_3);
  //dxl.ping(DXL_4);

  dxl.torqueOff(DXL_1);
  dxl.setOperatingMode(DXL_1, OP_POSITION);
  dxl.torqueOn(DXL_1);

  dxl.torqueOff(DXL_2);
  dxl.setOperatingMode(DXL_2, OP_POSITION);
  dxl.torqueOn(DXL_2);

  dxl.torqueOff(DXL_3);
  dxl.setOperatingMode(DXL_3, OP_POSITION);
  dxl.torqueOn(DXL_3);
  /*
  dxl.torqueOff(DXL_4);
  dxl.setOperatingMode(DXL_4, OP_POSITION);
  dxl.torqueOn(DXL_4);
  */
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_1, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_2, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_3, 0);
  //dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_4, 30);
  InverseKinematics(y_min);
}

void set180(){
  dxl.setGoalPosition(254, 180.00, UNIT_DEGREE);
}

void InverseKinematics(float y_data){
  float x = 0;
  float y = y_data;
  float cos_theta2 = (x*x+y*y-link1*link1-link2*link2)/(2*link1*link2);
  float sin_theta2 = sqrt(1-cos_theta2*cos_theta2);

  float theta1 = degrees(atan2(y,x)-atan2(link2*sin_theta2,link1+link2*cos_theta2))+90;
  float theta2 = degrees(atan2(sin_theta2, cos_theta2))+180;
  float theta3 = 540 - theta1 - theta2;

  move(theta1, theta2, theta3);
}

void move(float t1, float t2, float t3){
  float theta1 = t1+90;
  float theta2 = t2;
  float theta3 = t3;

  dxl.setGoalPosition(DXL_1, theta1, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_2, theta2, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_3, theta3, UNIT_DEGREE);
}

void lotation_move(){
  int y = y_min;
  for(int a = 0 ; a < 5000 ; a++){
  InverseKinematics(y);
  y= y+0.1;
  if(y >= 173.25) break;
  }
  delay(1000);
  for(int a = 0 ; a < 5000 ; a++){
  InverseKinematics(y);
  y=y-0.1;
  if(y <= 60) break;
  }
  delay(1000);
}

void loop() { 
  if(Serial2.available()){
    if(temp == 0) temp = 60;
    target = Serial2.parseInt();
    if( target > 0){
      Serial.read();
      Serial.print("target : "); Serial.println(target);
      Serial.print("temp : "); Serial.println(temp); 
    
      if(target > temp){
        Serial.print("Up count\n"); 
        for(int a = temp ; a < target+1 ; a ++){
          Serial.print(a); Serial.print(" "); 
          InverseKinematics(a);
        }
        Serial.println(); Serial.println();
      }
      else{
        Serial.print("Down count\n"); 
        for(int a = temp ; a > target-1 ; a --){
          Serial.print(a); Serial.print(" "); 
          InverseKinematics(a);
        }
        Serial.println(); Serial.println();
      }
      temp = target;
    }
  }
}