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

uint8_t initial_value = 0;
float flexSensorData[5] = {0};

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
  if (strcmp(peer_name, "Flex") == 0 || strcmp(peer_name, "tpad") == 0)
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
  if(strcmp(peer->name, "tpad") == 0)  {
    processtData(data);
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

void processtData(uint8_t* data) {
  // data[0] 에 action 값(1~5)이 들어옵니다.
  uint8_t actionValue = data[0];
  Serial.print("Received action: ");
  switch (actionValue) {
    case 1: break; //Right
    case 2: break; //Left
    case 3: command[1] = 1; break; //Back
    case 4: command[1] = 2; break; //Front
    case 5: command[1] = 0; break;
    default: break;
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

    Serial.write(0xEE);
    Serial.write(0xFF);
    for (int i = 0; i < 5; i++) {
      sendFloat(flexSensorData[i]);
    }
    sendFloat((float)command[0]);
    sendFloat((float)command[1]);
    sendFloat((float)command[2]);
    Serial.write("\n");

    //전송 후 command 초기화
    command[0] = 0;
    command[1] = 0;
    command[2] = 0;
  }
}
