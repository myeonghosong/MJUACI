#include <stdlib.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

int current = 0;
int SystemAllFlag =0;
int SystemDCmove = 0;
int SystemFlag = 99;
char mqttSysStep[] = "MJU/CD4/CHARGING/CAR";  //시스템 스텝

int step_target = 0;
int servo_target = 0;
int dc_target = 0;

char ssid[] = "MJU_WIFI";                              //WiFi ssid
char psw[] = "mjuwlan!";                   //Wifi password
char mqttSV[] = "cheese.mju.ac.kr";               //MQTT data
uint16_t mqttPN = 30220;                          //MQTT port number
char mqttTSNM[] = "TESTjaemin01";                 //MQTT test connect
char mqttTPz[] = "MJU/IOT/60181793/Z";  //스텝
char mqttTPy[] = "MJU/IOT/60181793/Y";  //DC
char mqttTPx[] = "MJU/IOT/60181793/X";  //서보

unsigned long long lastMS = 0;
unsigned long long lastMS2 = 0;

int xdata = 0;
int ydata = 0;
int zdata = 0;

int flagz = 0;
int flagy = 0;
int flagx = 0;

WiFiClient myTCPClient;
PubSubClient myMQTTClient;

void cbFunc(const char topic[], byte *data, unsigned int length){ //Callback Functiion
  char DATA[20] = {0,};     //For callback function
  memcpy(DATA, data, length);
  String buffz;
  String buffy;
  String buffx;

  if(topic[17] == 'Z') {
    buffz = DATA;
    zdata = buffz.toInt();
    Serial2.println(zdata);
  }
  if(topic[17] == 'Y'){
    int B;
    buffy = DATA;
  //<<<<<<< HEAD
    int b = 0;
    b = buffy.toInt();
    Serial.print("Y axis data input :"); Serial.println(b);
    if(b > 354){
      b = abs(354-b);
      b *= 0.4347826087;
      b += 20000;
      Serial.print("Go front : "); Serial.println(b);
      ydata = b;
    }
  //=======
    B = buffy.toInt();
    B = 354 - B;
    Serial.print("Y data : ");
    Serial.println(B);
    if(B>0){
      ydata = (B*0.4347826087) + 23000;
    }
    else if(B<0){
      B=abs(B);
      ydata = (B*0.4347826087) + 20000;
    }
    Serial.println(ydata);
    Serial2.println(ydata);
  //>>>>>>> 938744aac6d419de2c3c80f3fa7e759370eae655
  }
  if(topic[17] == 'X'){
    buffx = DATA;
    xdata = buffx.toInt();
    Serial1.println(xdata);
  }

  if (strcmp(topic, "MJU/CD4/CHARGING/CAR") == 0) {
    
    if (strcmp(DATA, "IN") == 0) { //차량 진입->해당위치로 이동 단계
        
        SystemAllFlag = 1;
        SystemDCmove = 21500; //커넥터 위치로 이동 값 (여유있게 줘서 가다가 수발광에 멈추도록?)
        SystemFlag = 0;
        current = 0;
        Serial.println("IN");
    }
    else if(strcmp(DATA, "FindCARNUM")==0){//번호판 인식단계
    
        SystemAllFlag = 2;
        current = 0;
        Serial.println("FindCARNUM");
        Serial2.println(10200);
    }
    else if(strcmp(DATA, "FindConnector")==0){//커넥터 찾아서 잡는 단계
        SystemAllFlag = 3;
        current = 0;
        Serial.println("FindConnector");
        Serial2.println(10100); //모터스피드 60
    }
    else if(strcmp(DATA, "GoToPort")==0){//차량 충전구로 이동 단계
        Serial2.println(13000); //모터스피드100
        SystemAllFlag = 4;
        current = 0;
        SystemDCmove = 23700; 
        SystemFlag = 0;
        Serial.println("GoToPort");
        
    }
    else if(strcmp(DATA, "FindPort")==0){//충전구 탐색 및 삽입 단계
        SystemAllFlag = 5;
        current = 0;
        Serial.println("FindPort");
        Serial2.println(10200);
    }
    else if(strcmp(DATA, "Success")==0){//충전연결 성공, 커넥터 꽂은채로 로봇팔 회수 단계
        Serial2.println(13000); //모터스피드100
        SystemAllFlag = 6;
        current = 0;
        Serial.println("Success");
    }
    else if(strcmp(DATA, "BackForWaiting")==0){//로봇 원래자리로 돌아가는 단계
        Serial2.println(13000); //모터스피드100
        SystemAllFlag = 7;
        current = 0;
        SystemDCmove = 23600;
        SystemFlag = 0;
        Serial.println("BackForWaiting");
    }
  
    else if(strcmp(DATA, "WaitingCharge")==0){//충전완료까지 대기하는 단계
        SystemAllFlag = 8;
        current = 0;
        Serial.println("WaitingCharge");
    }
    else if(strcmp(DATA, "StopCharging")==0){//충전정지신호 받고 로봇 충전구로 이동하는 단계
        Serial2.println(13000); //모터스피드100
        SystemAllFlag = 9;
        current = 0;
        SystemDCmove = 21000; //커넥터 (여유있게 줘서 가다가 수발광에 멈추도록?)
        SystemFlag = 0;
        Serial.println("StopCharging");
    }
    else if(strcmp(DATA, "PullConnector")==0){//커넥터 찾아서 잡고 전자서 켜서 뽑는 단계
        Serial2.println(10200);
        SystemAllFlag = 10;
        current = 0;
        Serial.println("PullConnector");
    }
    else if(strcmp(DATA, "PutConnector")==0){//커넥터 돌려놓으면서 회수시스템 작동 단계
        Serial2.println(13000); //모터스피드100
        SystemAllFlag = 11;
        current = 0;
        SystemDCmove = 23600; //커넥터 위치로 이동 값 (여유있게 줘서 가다가 수발광에 멈추도록?)
        SystemFlag = 0;
        Serial.println("PutConnector");
    }
    else if(strcmp(DATA, "CARINWaiting")==0){//원점으로 돌아가는 단계 끝나면 SystemAllFlag = 0으로 돌아가서 반복
        Serial2.println(13000); //모터스피드100
        SystemAllFlag = 12;
        current = 0;
        SystemDCmove = 24500; //커넥터 위치로 이동 값 (여유있게 줘서 가다가 수발광에 멈추도록?)
        SystemFlag = 0;
        Serial.println("CARINWaiting");
    }
    else if(strcmp(DATA, "RESET") == 0){ //전체 시스템 리셋
        SystemAllFlag = 99;
        SystemFlag = 0;
        Serial.println("RESET");
    }
    else if(strcmp(DATA, "TEST_F") == 0){
      Serial2.println(20300);
    }
    else if(strcmp(DATA, "TEST_B") == 0){
      Serial2.println(23300);
    }
    else if(strcmp(DATA, "TEST_R") == 0){
      Serial2.println(28888);
    }
    else if(strcmp(DATA, "TEST_L") == 0){
      Serial2.println(27777);
    }
    else if(strcmp(DATA, "Systemflag") == 0){
      current += 1;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 15, 2);
  Serial2.begin(115200, SERIAL_8N1, 12, 13);
  WiFi_START();
  MQTT_START();
}

void loop() {
  myMQTTClient.loop();

  if(Serial.available()>0){
    int v = Serial.parseInt();
    Serial2.println(v);
  }

  if(Serial2.available()>0){ // 우노가 수발광신호 받으면 다음스텝넘어가도록 플레그 쏴줌.
    int i = Serial2.parseInt();
    Serial.println(i);
    Serial.print("값:");
    Serial.println(current);
    if(i == 1){
      current += 1;
      SystemFlag = current;
    }
    Serial.println(SystemFlag);
  }
    if(Serial1.available()>0){ // 서보가 다음스텝넘어가도록 플레그 쏴줌.
    int i = Serial1.parseInt();
    if(i == 0){
      current += 1;
      SystemFlag = current;
    }
  }
    
    if(SystemAllFlag == 1){ //차량 진입->해당충전기 커넥터 위치로 이동 단계
        if(SystemFlag == 0){
            delay(300);
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 20600;
        }
        else if(SystemFlag == 1){
            delay(300);
            Serial2.println(SystemDCmove);
            SystemDCmove = 99;
        }
    }
    
    else if(SystemAllFlag == 4){ //차량 충전구로 이동 단계
        if(SystemFlag == 0){ //모서리까지 움직이기
            delay(300);
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 28888;
        }
        else if(SystemFlag == 1){// 코너 우회전
            delay(300);
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 20600;
        }
        else if(SystemFlag == 2 ){// 서보 반대로 돌리는단계 

            
        } 
        else if(SystemFlag == 3){ // 충전구까지 이동
            delay(300);
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
        }
    }
    else if(SystemAllFlag == 6){ //충전연결 성공, 커넥터 꽂은채로 로봇팔 회수 단계

    }
    else if(SystemAllFlag == 7){ //로봇 원래자리로 돌아가는 단계
        if(SystemFlag == 0){ //모서리까지 움직이기
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 27777;
        }
        else if(SystemFlag == 1){// 코너 우회전
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 23600;
        }
        else if(SystemFlag == 2){ // 커넥터위치
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 23600;
        }
        else if(SystemFlag == 3){ // 원점
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
        }
    }
    else if(SystemAllFlag == 9){ //충전정지신호 받고 로봇 충전구로 이동하는 단계
        if(SystemFlag == 0){
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 20500;
        }
        else if(SystemFlag == 1){ //모서리까지 움직이기
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 28888;
        }
        else if(SystemFlag == 2){// 코너 우회전
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 20600;
        }
        else if(SystemFlag == 3){ // 충전구까지 이동
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
        }
    }
    else if(SystemAllFlag == 11){ //커넥터 돌려놓으면서 회수시스템 작동 단계
        if(SystemFlag == 0){ //모서리까지 움직이기
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 27777;
        }
        else if(SystemFlag == 1){// 코너 좌회전
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
            SystemDCmove = 20600;
        }
        else if(SystemFlag == 2){ // 커넥터까지 이동
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
        }
        else if(SystemFlag == 3){
            // 서보로 뻗고 스텝으로 내려놓기
        }
    }
    else if(SystemAllFlag == 12){ //원점으로 돌아가는 단계 끝나면 SystemAllFlag = 0으로 돌아가서 반복
        if(SystemFlag == 0){ //모서리까지 움직이기
            Serial2.println(SystemDCmove);
            SystemFlag = 99;
        }
    }

    else if((SystemAllFlag == 3)||(SystemAllFlag == 5)||(SystemAllFlag ==10)){ // 커넥터 잡기 or 충전구 삽입
        if(millis() - lastMS > 2000){
            lastMS = millis();
            if(zdata <=500 && zdata >= 100 && flagz == 0){
            zdata = zdata + 30000;
            Serial2.println(zdata); 
            Serial.print("z axis data : "); Serial.println(zdata);
            }
            else if(zdata == 9999 && flagz == 0){
            Serial2.println(zdata);
            Serial.print("z axis data : "); Serial.println(zdata);
            flagz = 1;
            }
            if(ydata > 20000 && ydata < 20600 && flagy == 0){       
            Serial2.println(ydata);
            Serial.print("y axis data : "); Serial.println(ydata);
            }
            else if(ydata == 119999 && flagy == 0){
            Serial.print("y axis data : "); Serial.println(ydata);
            flagy = 1;
            }
            if(flagx == 0 && xdata > 50 && xdata < 1000){
            Serial.print("x axis data : "); Serial.println(xdata);
            //Serial1.println(xdata);
            flagx = 1;
            }
        }
    }
    else if(SystemAllFlag == 99){ // 시스템 초기화하는 코드(uno, 서보모터보드에 리셋 코드 보내주는 단계)
        
    }
}


void MQTT_START(){  //MQTT Start
  myMQTTClient.setClient(myTCPClient);
  myMQTTClient.setServer(mqttSV,mqttPN);
  myMQTTClient.setCallback(cbFunc);
  int result = myMQTTClient.connect(mqttTSNM);
  myMQTTClient.subscribe(mqttTPz);
  myMQTTClient.subscribe(mqttTPx);
  myMQTTClient.subscribe(mqttTPy);
  myMQTTClient.subscribe(mqttSysStep);
  
  printf("MQTT Connected. Result : %d\r\n",result);
}

void WiFi_START(){  //WiFi Start
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(ssid,psw);

  while(1){
    if(WiFi.waitForConnectResult() == WL_CONNECTED){
      printf("Connected!\r\n");
      break;
    }
    else{
      printf("Something Wrong\r\n");
      while(1) sleep(1);
    }
  }
}