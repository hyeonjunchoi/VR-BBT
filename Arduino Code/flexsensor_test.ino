//구부림센서(flex)의 측정값 테스트하는 프로그램
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("1,2,3,4,5");
}

void loop() {
  // put your main code here, to run repeatedly:
  int sensor_1 = analogRead(A0); //엄지 손가락
  int sensor_2 = analogRead(A1); //검지
  int sensor_3 = analogRead(A2); //중지
  int sensor_4 = analogRead(A3);
  int sensor_5 = analogRead(A4); //새끼손가락

  int hand_data = 0;


  Serial.print(sensor_1);
  Serial.print(",");
  Serial.print(sensor_2);
  Serial.print(",");
  Serial.print(sensor_3);
  Serial.print(",");
  Serial.print(sensor_4);
  Serial.print(",");
  Serial.println(sensor_5);


  delay(100);
}