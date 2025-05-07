#include <bluefruit.h>
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

#define motorPin1 2
#define motorPin2 3
#define valvePin1 4
#define valvePin2 5
#define ALPHA 0.8

int command[3] = {0};

typedef struct
{
  char name[16+1];
  uint16_t conn_handle;
  BLEClientUart bleuart;
} prph_info_t;

#define BLE_MAX_CONNECTION 2
prph_info_t prphs[BLE_MAX_CONNECTION];

uint8_t connection_num = 0;

float An[3], a[3];
uint8_t buttonVal;
uint8_t initial_value = 0;
float Origin[3] = {0};
bool reset = false;
float flexSensorData[5] = {0};

// IMU 데이터의 경우, 두 종류의 데이터 형식이 있음
// type 0: sys, gyro, accel, mag
// type 1: 계산된 angle과 command
volatile uint8_t imuDataType = 255; // 0 또는 1
float imu_sys = 0, imu_gyro = 0, imu_accel = 0, imu_mag = 0;
float imu_angle[3] = {0, 0, 0};

void setup() 
{
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(valvePin1, OUTPUT);
  pinMode(valvePin2, OUTPUT);
  Serial.begin(115200);
  
  while ( !Serial ) delay(10);

  Bluefruit.begin(0, BLE_MAX_CONNECTION);
  Bluefruit.setName("Cent");
  
  for (uint8_t idx=0; idx<BLE_MAX_CONNECTION; idx++)
  {
    prphs[idx].conn_handle = BLE_CONN_HANDLE_INVALID;
    prphs[idx].bleuart.begin();
    prphs[idx].bleuart.setRxCallback(bleuart_rx_callback);
  }

  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);
  Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE);
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);
}

void scan_callback(ble_gap_evt_adv_report_t* report)
{
  char peer_name[32] = { 0 };
  Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, (uint8_t*)peer_name, sizeof(peer_name));
  if (strcmp(peer_name, "Flex") == 0 || strcmp(peer_name, "Head") == 0)
  {
    Serial.print("Found target device: ");
    Serial.println(peer_name);
    Bluefruit.Central.connect(report);
  }
}

void connect_callback(uint16_t conn_handle)
{
  int id  = findConnHandle(BLE_CONN_HANDLE_INVALID);
  if ( id < 0 ) return;
  
  prph_info_t* peer = &prphs[id];
  peer->conn_handle = conn_handle;
  
  Bluefruit.Connection(conn_handle)->getPeerName(peer->name, sizeof(peer->name)-1);
  Serial.print("Connected to ");
  Serial.println(peer->name);

  if ( peer->bleuart.discover(conn_handle) )
  {
    peer->bleuart.enableTXD();
    Bluefruit.Scanner.start(0);
  } 
  else
  {
    Bluefruit.disconnect(conn_handle);
  }

  connection_num++;
  if (connection_num < BLE_MAX_CONNECTION) {
    Serial.println("Scanning for more devices...");
    Bluefruit.Scanner.start(0);
  }
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) reason;
  connection_num--;

  int id  = findConnHandle(conn_handle);
  if ( id < 0 ) return;
  prphs[id].conn_handle = BLE_CONN_HANDLE_INVALID;

  Serial.print(prphs[id].name);
  Serial.println(" disconnected!");

  if (connection_num == 0 && !Bluefruit.Scanner.isRunning()) {
    Serial.println("All devices disconnected. Restarting scan...");
    Bluefruit.Scanner.start(0);
  }
}

#define MAX_DATA_SIZE 20
void bleuart_rx_callback(BLEClientUart& uart_svc)
{
  uint16_t conn_handle = uart_svc.connHandle();
  int id = findConnHandle(conn_handle);
  prph_info_t* peer = &prphs[id];
    
  byte data[MAX_DATA_SIZE] = {0};
  size_t dataLength = 0;

  while (uart_svc.available() && dataLength < MAX_DATA_SIZE)  {
    data[dataLength++] = uart_svc.read();
  }
  if(strcmp(peer->name, "Flex") == 0)  {
    processFlexData(data);
  }
  if(strcmp(peer->name, "Head") == 0)  {
    processIMUData(data, An, a, &buttonVal);
  }
}

void processFlexData(byte* data) {
  uint16_t sensorVals[5];

  for (int i = 0, j = 0; i < 5; i++, j += 2) {
    sensorVals[i] = (data[j] << 8) | data[j + 1];
  }
  // 즉시 전송하지 않고 flex 센서 데이터를 전역 변수에 저장
  for (int i = 0; i < 5; i++) {
    flexSensorData[i] = (float)sensorVals[i];
  }
}

void processIMUData(byte* data, float An[3], float a[3], uint8_t* buttonVal) {
    uint8_t sys = 0;
    uint8_t gyro = 0;
    uint8_t accel = 0;
    uint8_t mag = 0;
    
    if(data[0] == 0){
      initial_value = data[0];
      sys = data[1];
      gyro = data[2];
      accel = data[3];
      mag = data[4];
      
      // IMU type 0 데이터 저장
      imuDataType = 0;

      imu_sys = (float)sys;
      imu_gyro = (float)gyro;
      imu_accel = (float)accel;
      imu_mag = (float)mag;
    }
    else if(data[0] == 1){
      initial_value = data[0];
      int index = 1;
      *buttonVal = (uint8_t)data[index++];
      for (int i = 0; i < 3; i++) {
          int16_t decompressed = (data[index++] << 8) | data[index++];
          An[i] = decompressed / 10.0;
      }
      for (int i = 0; i < 3; i++) {
          int16_t decompressed = (data[index++] << 8) | data[index++];
          a[i] = decompressed / 100.0;
      }

      if(*buttonVal == 1){
        Origin[0] = An[0];
        Origin[1] = An[1];
        Origin[2] = An[2];
        reset = true;
      }
      else{
        // 이전 값(글로벌 imu_angle)을 이용하여 low pass filter 적용
        imu_angle[0] = ALPHA * (Origin[0] - An[0]) + (1 - ALPHA) * imu_angle[0];
        imu_angle[1] = ALPHA * (Origin[1] - An[1]) + (1 - ALPHA) * imu_angle[1];
        imu_angle[2] = ALPHA * (Origin[2] - An[2]) + (1 - ALPHA) * imu_angle[2];
      }
      
      // 계산된 angle 값으로 command 업데이트
      imuDataType = 1;
      for (int i = 0; i < 3; i++) {
        if (imu_angle[i] > 20) {
          command[i] = 1;
        }
        else if (imu_angle[i] < -20) {
          command[i] = 2;
        }
        else{
          command[i] = 0;
        }
      }
    }
}

void sendFloat(float value) {
  Serial.write((uint8_t*)&value, sizeof(float));
}

int findConnHandle(uint16_t conn_handle)
{
  for(int id = 0; id < BLE_MAX_CONNECTION; id++)
  {
    if (prphs[id].conn_handle == conn_handle)
    {
      return id;
    }
  }
  return -1;  
}

void loop()
{
  static uint32_t lastPrintTime = 0;
  if (millis() - lastPrintTime >= 200) {
    lastPrintTime = millis();
    
    if(reset == true){ //reset 버튼이 눌릴 때부터 모터 컨트롤
      if (command[1] == 1) { // 손 피기
        analogWrite(motorPin1, 255);
        analogWrite(motorPin2, 255);
        analogWrite(valvePin1, 255);
        analogWrite(valvePin2, 0);
      }
      else if (command[1] == 2) { // 손 접기
        analogWrite(motorPin1, 255);
        analogWrite(motorPin2, 255);
        analogWrite(valvePin1, 0);
        analogWrite(valvePin2, 255);
      }
      else { // 손 고정
        analogWrite(motorPin1, 0);
        analogWrite(motorPin2, 0);
        analogWrite(valvePin1, 0);
        analogWrite(valvePin2, 0);
      }
    }
    else{
        analogWrite(motorPin1, 0);
        analogWrite(motorPin2, 0);
        analogWrite(valvePin1, 0);
        analogWrite(valvePin2, 0);
    }

    if(imuDataType == 0) {
      Serial.write(0xAA);
      Serial.write(0xBB);
      for (int i = 0; i < 5; i++) {
        sendFloat(flexSensorData[i]);
      }
      sendFloat(imu_sys);
      sendFloat(imu_gyro);
      sendFloat(imu_accel);
      sendFloat(imu_mag);
      Serial.write("\n");
    }
    else if(imuDataType == 1) {
      Serial.write(0xCC);
      Serial.write(0xDD);
      for (int i = 0; i < 5; i++) {
        sendFloat(flexSensorData[i]);
      }
      sendFloat(imu_angle[0]);
      sendFloat(imu_angle[1]);
      sendFloat(imu_angle[2]);
      sendFloat((float)command[0]);
      sendFloat((float)command[1]);
      sendFloat((float)command[2]);
      Serial.write("\n");
    }
  }
}
