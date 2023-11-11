#include <SoftwareSerial.h>

#define STEP_DELAY 700
#define STEP_PIN 2
#define DIR_PIN 3
#define ENA_PIN 4
#define motorpin1 5
#define motorpin2 6
#define motorpin3 10
#define motorpin4 11


SoftwareSerial mySerial(13,12);

int target_pix = 0;
int target_mm = 0;
int target_temp = 0;
int target_value = 0;
int target_delay = 0;
int target_check1 = 0;
int target_check2 = 0;
int current_value = 0;
int target_middle = map(240, 480, 0, 50, 300);
int target_a = 0;
int temp = 0;

int flagy = 1;
int flagz = 0;

void setup()
{
  Serial.begin(115200);
  mySerial.begin(115200);
  Serial.println("Serial connected!");
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(ENA_PIN, HIGH);
  delay(100);
  target_mm = 200;
  target_temp = target_mm;
  target_value = (target_temp - 50) * 20;
  Serial.print("target to : "); Serial.println(target_value);
  delay(10);
  step_move();



  while(1){
  if(mySerial.available()>0){   
    target_pix = mySerial.parseInt();
    Serial.print("Data income : "); Serial.println(target_pix);
    if(target_pix != 0){
      if(target_pix < 19999){
        target_check1 = target_pix;
        target_pix = 240 - target_pix;
        target_mm = target_pix * 0.33;
        Serial.print("check : "); Serial.print(target_check1);
        Serial.print("  |   pixel : "); Serial.println(target_pix);
        if(target_mm < 500 && target_mm > -500){
          Serial.print("offset : "); Serial.print(target_mm);

          target_temp += target_mm;

          Serial.print("  |   current : "); Serial.print(target_temp);
          target_value = (target_temp - 50) * 20;

          Serial.print("  |   target_v : "); Serial.println(target_value);
          if(target_value <= 0) target_value = 0;
          else if(target_value >= 5340) target_value = 5340;
          step_move();
        }
        else if (target_check1 == 9999){
          Serial.println("Z axis success!!");
          target_temp += 53;
          Serial.print("current : "); Serial.println(target_temp);
          target_value = (target_temp - 50) * 20;
          step_move();
          flagz = 0;
        } 
      }
      else if(target_pix >= 20000){
        target_check2 = target_pix;
        target_pix -= 20000;
        target_pix  = 354 - target_pix;
        target_mm   = target_pix * 0.4347826087;
        target_delay = abs(target_mm)* 10;
        Serial.print("check : "); Serial.print(target_check2);
        Serial.print("  |   pixel : "); Serial.print(target_pix);
        Serial.print("  |   goal  : "); Serial.print(target_mm);
        Serial.print("  |   delay : "); Serial.println(target_delay);

        if(target_mm < 0){
          dc_forward();
          delay(50);
          dc_stop();
        }
        else if(target_mm > 0 && target_mm < 90000){
          dc_backward();
          delay(50);
          dc_stop();
        }
        else if(target_check2 == 99999){
          dc_stop();
          flagy = 0;
          flagz = 1;
        }
      }
    }
  }
  }
}

void loop(){
}


void step_move(){
  while ( target_value > current_value ){ 
    step_forward();  
  }
  while ( target_value < current_value ){ 
    step_backward(); 
  }
  digitalWrite(ENA_PIN, HIGH); 
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
  analogWrite(motorpin1, 60);
  analogWrite(motorpin2, 0);
  analogWrite(motorpin3, 60);
  analogWrite(motorpin4, 0);
  Serial.println("Forward");
}

void dc_backward(){
  analogWrite(motorpin1, 0);
  analogWrite(motorpin2, 60);
  analogWrite(motorpin3, 0);
  analogWrite(motorpin4, 60);
  Serial.println("Backward");

}

void dc_left(){
  analogWrite(motorpin1, 60);
  analogWrite(motorpin2, 0);
  analogWrite(motorpin3, 0);
  analogWrite(motorpin4, 60);
  Serial.println("left");
}

void dc_right(){
  analogWrite(motorpin1, 0);
  analogWrite(motorpin2, 60);
  analogWrite(motorpin3, 60);
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