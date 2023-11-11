#include <SoftwareSerial.h>

#define STEP_DELAY 700

#define DIR_PIN 3
#define STEP_PIN 2
#define ENA_PIN 4
#define motorpin1 5
#define motorpin2 6
#define motorpin3 10
#define motorpin4 11

SoftwareSerial mySerial(13,12);

int target_value = 0;
int current_value = 0;
int temp = 0;

void setup()
{
  Serial.begin(115200);
  mySerial.begin(115200);
  motor_setup();
}

void loop()
{
  if ( mySerial.available() )
  {
    target_value = mySerial.parseInt();
    Serial.print("Input data = ");
    Serial.println(target_value);
    if(target_value != 0){
      if(target_value == 12345) target_value = 0;
      if(target_value >= -5000 && target_value <= 5000) step_move();
      if(target_value == 10001) {
        dc_forward(); 
        delay(1200);
        dc_stop();
      }
      if(target_value == 10002) {
        dc_backward(); 
        delay(1200);
        dc_stop();
      }
      if(target_value == 10003) {
        dc_left(); 
        delay(1200);
        dc_stop();
      }
      if(target_value == 10004) {
        dc_right(); 
        delay(1200);
        dc_stop();
      }
      if(target_value == 10005) {
        dc_stop(); 
        delay(1200);
      }
      delay(10);
    }
    digitalWrite(ENA_PIN, HIGH);
  }

  Serial.print("TARGET : ");
  Serial.print(target_value);
  Serial.print(" , CUR : ");
  Serial.print(current_value);
  Serial.println("");
}


void motor_setup(){
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(ENA_PIN, HIGH);
  pinMode(motorpin1, OUTPUT);
  pinMode(motorpin2, OUTPUT);
  pinMode(motorpin3, OUTPUT);
  pinMode(motorpin4, OUTPUT);
}
void step_move(){
    while ( target_value > current_value )
    {
      step_forward();
    }
    while ( target_value < current_value )
    {
      step_backward();
    }
}

void step_forward()
{
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY);
  current_value++;
}

void step_backward()
{
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY);
  current_value--;
}

void dc_forward(){
  analogWrite(motorpin1, 100);
  analogWrite(motorpin2, 0);
  analogWrite(motorpin3, 100);
  analogWrite(motorpin4, 0);
  Serial.println("Forward");
}

void dc_backward(){
  analogWrite(motorpin1, 0);
  analogWrite(motorpin2, 100);
  analogWrite(motorpin3, 0);
  analogWrite(motorpin4, 100);
  Serial.println("Backward");

}

void dc_left(){
  analogWrite(motorpin1, 100);
  analogWrite(motorpin2, 0);
  analogWrite(motorpin3, 0);
  analogWrite(motorpin4, 100);
  Serial.println("left");
}

void dc_right(){
  analogWrite(motorpin1, 0);
  analogWrite(motorpin2, 100);
  analogWrite(motorpin3, 100);
  analogWrite(motorpin4, 0);
  Serial.println("right");
}


void dc_stop(){
  analogWrite(motorpin1, 0);
  analogWrite(motorpin2, 0);
  analogWrite(motorpin3, 0);
  analogWrite(motorpin4, 0);
  Serial.println("stop");
}