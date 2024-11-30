#include <Wire.h>

int motorPin1 = 3;
int motorPin2 = 4;
int valvePin1 = 5;
int valvePin2 = 6;
int sensorPins[5] = {A0, A1, A2, A3, A4}; // 5개의 아날로그 센서 핀 배열

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(valvePin1, OUTPUT);
  pinMode(valvePin2, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // 첫 번째 상황: 손 접기 (1)
  Serial.println("0");  // 손 접기 시작 -> 1
  analogWrite(motorPin1, 255);
  analogWrite(motorPin2, 255);
  analogWrite(valvePin1, 0);
  analogWrite(valvePin2, 255);

  unsigned long startTime = millis();
  while (millis() - startTime < 4000) {
    for (int i = 0; i < 5; i++) {
      int sensorValue = analogRead(sensorPins[i]);
      Serial.print(sensorValue);
      if (i < 4) Serial.print(", ");
    }
    Serial.println();
    delay(100);
  }

  // 두 번째 상황: 손 접은 상태로 고정 (1)
  //Serial.println("0");  // 손 접은 상태로 고정 시작 -> 1
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(valvePin1, 0);
  analogWrite(valvePin2, 0);

  startTime = millis();
  while (millis() - startTime < 5000) {
    for (int i = 0; i < 5; i++) {
      int sensorValue = analogRead(sensorPins[i]);
      Serial.print(sensorValue);
      if (i < 4) Serial.print(", ");
    }
    Serial.println();
    delay(100);
  }
  Serial.println(3);

  // 세 번째 상황: 손 피기 (0)
  Serial.println("1");  // 손 피기 시작 -> 0
  analogWrite(motorPin1, 255);
  analogWrite(motorPin2, 255);
  analogWrite(valvePin1, 255);
  analogWrite(valvePin2, 0);

  startTime = millis();
  while (millis() - startTime < 4000) {
    for (int i = 0; i < 5; i++) {
      int sensorValue = analogRead(sensorPins[i]);
      Serial.print(sensorValue);
      if (i < 4) Serial.print(", ");
    }
    Serial.println();
    delay(100);
  }

  // 네 번째 상황: 손 핀 상태로 고정 (0)
  //Serial.println("1");  // 손 핀 상태로 고정 시작 -> 0
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(valvePin1, 0);
  analogWrite(valvePin2, 0);

  startTime = millis();
  while (millis() - startTime < 5000) {
    for (int i = 0; i < 5; i++) {
      int sensorValue = analogRead(sensorPins[i]);
      Serial.print(sensorValue);
      if (i < 4) Serial.print(", ");
    }
    Serial.println();
    delay(100);
  }
}
