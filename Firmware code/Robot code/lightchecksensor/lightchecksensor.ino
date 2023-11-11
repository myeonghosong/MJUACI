int carsensor;

void setup() {
  Serial.begin(115200);
  pinMode(A0,INPUT);
}

void loop() {
  carsensor = analogRead(A0);
  Serial.println(carsensor);
  delay(200);
}
