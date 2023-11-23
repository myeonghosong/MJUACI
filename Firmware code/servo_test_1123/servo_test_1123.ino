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
float temp_y = 60;
float temp_x = 0;
int servo_flag = 0;

const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void Inverse_Kinematics(float x_data, float y_data){
  float x = x_data;
  float y = y_data;
  float cos_theta2 = (x*x+y*y-link1*link1-link2*link2)/(2*link1*link2);
  float sin_theta2 = sqrt(1-cos_theta2*cos_theta2);

  float theta1 = degrees(atan2(y,x)-atan2(link2*sin_theta2,link1+link2*cos_theta2))+90;
  float theta2 = degrees(atan2(sin_theta2, cos_theta2))+180;
  float theta3 = 0;
  int angle_flag = 1;

  if(theta1 >= 0 && theta1 < 90 && angle_flag == 1){
    theta1 = 180 - theta1;
    theta2 = 360 - theta2;
    theta3 = 450 - theta1 - theta2;
    angle_flag = 0;
  }

  if(theta1 >= 90 && theta1 < 180 && angle_flag == 1){
    if(x_data == 0){ // 정면으로 뻗기
      theta3 = 540 - theta1 - theta2;
    }
    else if(y_data == 0){ // 오른쪽으로 뻗기
      theta3 = 450 - theta1 - theta2;
    }
    angle_flag = 0;
  }
  else if(theta1 >= 180 && theta1 < 270 && angle_flag == 1){
    if(x_data == 0){ // 정면으로 뻗기
      theta3 = 540 + theta1 - theta2;
    }
    else if(y_data == 0){ // 왼쪽으로 뻗기
      theta3 = 675 - theta1 - theta2;
    }
    angle_flag = 0;
  }

  else if(theta1 >= 270 && theta1 < 360 && angle_flag == 1){
    theta1 = 540 - theta1;
    theta2 = 360 - theta2;
    theta3 = 675 - theta1 - theta2;
    angle_flag = 0;
  }

  dxl.setGoalPosition(DXL_1, theta1, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_2, theta2, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_3, theta3, UNIT_DEGREE);
}

void f_to_f(float x_data, float y_data){
  Serial.println("f_to_f");
  delay(10);
  float x = x_data;
  float y = y_data;
  if(y > temp_y){  // 팔 앞으로 뻗기  x : 0 / y : +
    for(int count = temp_y ; count < y + 1 ; count ++){
      Serial.print("x : "); Serial.print("0"); Serial.print(" y : "); Serial.println(count);
      int target_x = 0;
      int target_y = count;
      Inverse_Kinematics(target_x, target_y);
    }
  }
  else if(y < temp_y){  // 팔 뒤로 당기기 x : 0 / y : -
    for(int count = temp_y ; count > y - 1 ; count --){
      Serial.print("x : "); Serial.print("0"); Serial.print(" y : "); Serial.println(count);
      int target_x = 0;
      int target_y = count;
      Inverse_Kinematics(target_x, target_y);
    }
  }
  temp_x = x;
  temp_y = y;
}

void l_to_f(float x_data, float y_data){  // x : 음수 -> 0 / y : 0 -> 목표
  Serial.println("l_to_f");
  delay(10);
  float x = x_data;
  float y = y_data;
  float count_x = temp_x;
  float count_y = 0;
  while(count_x < 0 && count_y < y){ // x가 음수이고 y가 목표값보다 작을 동안
    Inverse_Kinematics(count_x, count_y);
    Serial.print("x : "); Serial.print(count_x); Serial.print(" y : "); Serial.println(count_y);
    if(count_x < 0) count_x ++;
    if(count_y < y) count_y ++;
  }
  temp_x = x;
  temp_y = y;
}

void r_to_f(float x_data, float y_data){  // x : 양수 -> 0 / y : 0 -> 목표
  Serial.println("r_to_f");
  delay(10);
  float x = x_data;
  float y = y_data;
  float count_x = temp_x;
  float count_y = 0;
  while(count_x > 0 && count_y < y){ // x가 양수이고 y가 목표값보다 작을 동안
    Inverse_Kinematics(count_x, count_y);
    Serial.print("x : "); Serial.print(count_x); Serial.print(" y : "); Serial.println(count_y);
    if(count_x > 0) count_x --;
    if(count_y < y) count_y ++;
  }
  temp_x = x;
  temp_y = y;
}

void l_to_l(float x_data, float y_data){
  Serial.println("l_to_l");
  delay(10);
  float x = x_data;
  float y = y_data;
  if(y > temp_y){  // 팔 앞으로 뻗기  x : - / y : 0
    for(int count = temp_x ; count > x - 1 ; count --){ //ex) x = 0 to x = -100
      Serial.print("x : "); Serial.print(count); Serial.print(" y : "); Serial.println(y);
      int target_x = count;
      int target_y = y;
      Inverse_Kinematics(target_x, target_y);
    }
  }
  else if(y < temp_y){  // 팔 뒤로 당기기 x : + / y : 0
    for(int count = temp_x ; count < x + 1 ; count ++){ // ex) x = -100 to x = 0
      Serial.print("x : "); Serial.print(count); Serial.print(" y : "); Serial.println(y);
      int target_x = count;
      int target_y = y;
      Inverse_Kinematics(target_x, target_y);
    }
  }
  temp_x = x;
  temp_y = y;
}

void f_to_l(float x_data, float y_data){ // x : 0 -> 음수 목표 / y : 양수 -> 0
  Serial.println("f_to_l");
  delay(10);
  float x = x_data;
  float y = y_data;
  float count_x = 0;
  float count_y = temp_y;
  while(count_x > x && count_y > 0){ // x가 목표값보다 크고 y가 양수인 동안  /  ex) x = 0 to -100 y = 100 to 0
    Inverse_Kinematics(count_x, count_y);
    Serial.print("x : "); Serial.print(count_x); Serial.print(" y : "); Serial.println(count_y);
    if(count_x > x) count_x --;
    if(count_y > 0) count_y --;
  }
  temp_x = x;
  temp_y = y;  
}

void r_to_r(float x_data, float y_data){
  Serial.println("r_to_r");
  delay(10);
  float x = x_data;
  float y = y_data;
  if(y > temp_y){  // 팔 앞으로 뻗기  x : + / y : 0
    for(int count = temp_x ; count < x + 1 ; count ++){ // ex) x = 0 to x = 100
      Serial.print("x : "); Serial.print(count); Serial.print(" y : "); Serial.println(y);
      int target_x = count;
      int target_y = y;
      Inverse_Kinematics(target_x, target_y);
    }
  }
  else if(y < temp_y){  // 팔 뒤로 당기기 x : - / y : 0
    for(int count = temp_x ; count > x - 1 ; count --){ // ex) x = 100 to x = 0
      Serial.print("x : "); Serial.print(count); Serial.print(" y : "); Serial.println(y);
      int target_x = count;
      int target_y = y;
      Inverse_Kinematics(target_x, target_y);
    }
  }
  temp_x = x;
  temp_y = y;  
}

void f_to_r(float x_data, float y_data){ // x : 0 -> 양수 목표 / y : 양수 -> 0
  Serial.println("f_to_r");
  delay(10);
  float x = x_data;
  float y = y_data;
  float count_x = 0;
  float count_y = temp_y;
  Serial.print("목표 : "); Serial.print(x); Serial.print(" / "); Serial.print(y); Serial.print("현재 : "); Serial.print(count_x); Serial.print(" / "); Serial.println(count_y);
  while(count_x < x && count_y > 0){ // x가 목표값보다 작고 y가 양수인 동안  /  ex) x = 0 to 100 y = 100 to 0
    Inverse_Kinematics(count_x, count_y);
    Serial.print("x : "); Serial.print(count_x); Serial.print(" y : "); Serial.println(count_y);
    if(count_x < x) count_x ++;
    if(count_y > 0) count_y --;
  }
  temp_x = x;
  temp_y = y;    
}

void moving_front(float x_data, float y_data){
  float x = x_data;
  float y = y_data;
  if(temp_x == 0 && x == 0){  // 팔 뻗기
    Serial.println("move front");
    delay(10);
    f_to_f(0, y);
  }
  else if(temp_x < 0 && x == 0){ // 팔 왼쪽에서 중앙으로 x : - / y : + 
    Serial.println("move left to front");
    delay(10);
    l_to_f(0, 60);
    delay(10);
    f_to_f(0, y);
  }
  else if(temp_x > 0 && x == 0){ // 팔 오른쪽에서 중앙으로  x : - / y : +
    Serial.println("move right to front");
    delay(10);
    r_to_f(0, 60);
    delay(10);
    f_to_f(0, y);
  }
}

void moving_left(float x_data, float y_data){
  float x = x_data;
  float y = y_data;
  if(y == 0 && temp_y == 0){ // 팔이 왼쪽에 있을 경우
    Serial.println("move left");
    delay(10);
    l_to_l(x, 0);     // 왼쪽 -> 뻗기
  }
  if(y == 0 && temp_y > 0){ 
    if(temp_x == 0){ // 팔이 중앙일 경우 x : + / y : -
      Serial.println("move front to left");
      delay(10);
      f_to_l(60, 0);  // 중앙 -> 왼쪽
      delay(10);
      l_to_l(x, 0);   // 왼쪽 -> 뻗기
    }
    if(temp_x > 0){ // 팔이 오른쪽에 있을 경우
      Serial.println("move right to left");
      delay(10);
      r_to_f(0, 60);  // 오른쪽 -> 중앙
      delay(10);
      f_to_l(60, 0);  // 중앙 -> 왼쪽
      delay(10);
      l_to_l(x, 0);   // 왼쪽 -> 뻗기
    }
  }
}

void moving_right(float x_data, float y_data){
  float x = x_data;
  float y = y_data;
  if(y == 0 && temp_y == 0){ // 팔이 오른쪽에 있을 경우
    Serial.println("move right");
    delay(10);
    r_to_r(x, 0);    // 오른쪽 -> 뻗기
  }
  if(y == 0 && temp_y > 0){ // 팔이 중앙일 경우
    if(temp_x == 0){
      Serial.println("move front to right");
      delay(10);
      f_to_r(60, 0); // 중앙 -> 오른쪽
      delay(10);
      r_to_r(x, 0);  // 오른쪽 -> 뻗기
    }
    if(temp_x < 0){  // 팔이 왼쪽에 있을 경우
      Serial.println("move left to front");
      delay(10);
      l_to_f(0,60);  // 왼쪽 -> 중앙
      delay(10);
      f_to_r(60, 0); // 중앙 -> 오른쪽
      delay(10);
      r_to_r(x, 0);  // 오른쪽 -> 뻗기
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_1);
  dxl.ping(DXL_2);
  dxl.ping(DXL_3);

  dxl.torqueOff(DXL_1);
  dxl.setOperatingMode(DXL_1, OP_POSITION);
  dxl.torqueOn(DXL_1);

  dxl.torqueOff(DXL_2);
  dxl.setOperatingMode(DXL_2, OP_POSITION);
  dxl.torqueOn(DXL_2);

  dxl.torqueOff(DXL_3);
  dxl.setOperatingMode(DXL_3, OP_POSITION);
  dxl.torqueOn(DXL_3);

  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_1, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_2, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_3, 0);

  moving_front(0,60);
}

void loop() { 
  if(Serial.available()){
    target = Serial.parseInt();
    if( target > 0 && target < 1000){ // 로봇팔 중앙으로
      Serial.print("Front / target : "); Serial.println(target);
      delay(100);
      Inverse_Kinematics(0, target);
    }
    else if(target > 1000 && target < 2000){ // 로봇 팔 왼쪽으로
      target -= 1000;
      target *= -1;
      Serial.print("Left / target : "); Serial.println(target);
      delay(100);
      Inverse_Kinematics(target, 0);
    }
    else if(target > 2000 && target < 3000){ // 로봇 팔 오른쪽으로
      target -= 2000;
      Serial.print("Right / target : "); Serial.println(target);
      delay(100);
      Inverse_Kinematics(target, 0);
      
    }
  }
}
