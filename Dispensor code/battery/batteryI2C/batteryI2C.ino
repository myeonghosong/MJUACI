#include <stdlib.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <HardwareSerial.h>

WiFiClient myTCPClient;
PubSubClient myMQTTClient;
HardwareSerial dfSerial(2);

char* yes ="y";
char* no = "n";
char s1;
unsigned long int change;
void cbFunc(const char topic[],byte *data, unsigned int length){
  char str[80] = {0,};
  memcpy(str, data, length);

  if (strcmp(topic, "MJU/CD4/CHARGING") == 0) {
    if (strcmp(str, "READY2") == 0) {
    change = 1;
  }
    else if(strcmp(str, "FINISH")==0){
    change = 0;
  }
    else if(strcmp(str, "CHARGINGSTART")==0){
    change = 2;
    Serial2.print(yes);
  }
    else if(strcmp(str, "CHARGINGSTOP")==0){
    change = 3;
    Serial2.print(no);
  }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  dfSerial.begin(9600,SERIAL_8N1,16,17);
  Serial.print("start");
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin("MJU_WIFI","mjuwlan!");
  while(1)
  {
    if(WiFi.waitForConnectResult()==WL_CONNECTED){
      printf ("Connected!\r\n"); break;}

    else{printf("Something Wrong\r\n"); while(1) sleep(1);}
  }
  Wire.begin(0x13);
  Wire.onReceive(receiveEvent);

  myMQTTClient.setClient(myTCPClient);
  myMQTTClient.setServer("cheese.mju.ac.kr",30220);
  myMQTTClient.setCallback(cbFunc);
  int result = myMQTTClient.connect("60181793mh");
  myMQTTClient.subscribe("MJU/CD4/CHARGING");
  printf("MQTT Conn.. Result:%d\r\n",result);
}

void loop() {
  myMQTTClient.loop();
}

void receiveEvent(int parameter){
  while(Wire.available()){

    if(change == 0){Serial.print("not ready");}
    else if(change == 2){
      Serial.print("START -> ");
     // Serial2.println(47);
    }
        int a = Wire.read();
    char s1[10];
    Serial.print(a);
    Serial.println("%");
    snprintf(s1,sizeof(s1), "%d", a);
    myMQTTClient.publish("MJU/CD4/CHARGING/PER",s1);
  }
}