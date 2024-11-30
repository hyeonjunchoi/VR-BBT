#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <bluefruit.h>

// BLE Service
BLEUart bleuart; // uart over ble

// IMU Sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Timer for reading IMU data
SoftwareTimer IMUTimer;

// BLE device name
const char* device_name = "IMU_Device";

// Sampling rate for IMU
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

void setup(void)
{
  Wire.begin();
  Serial.begin(115200);

  // IMU Sensor Initialization
  if (!bno.begin()) {
    Serial.println("No BNO055 detected");
    while (1) {
      delay(100);
    }
  }
  bno.setExtCrystalUse(true);

  // BLE Initialization
  Serial.println("Initializing BLE...");
  Bluefruit.begin();
  Bluefruit.setName(device_name);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Start BLE UART Service
  bleuart.begin();

  // Set up and start advertising
  startAdv();

  Serial.println("Please use a BLE app to connect in UART mode");

  // Start the IMU timer
  IMUTimer.begin(BNO055_SAMPLERATE_DELAY_MS, IMUTimer_callback);
  IMUTimer.start();
}

void loop(void)
{
  // No tasks needed in the main loop
}

void IMUTimer_callback(TimerHandle_t xTimerID)
{
  // Read Euler angles from BNO055
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float heading = euler.x();
  float roll = euler.y();
  float pitch = euler.z();

  // Print values to Serial
  Serial.print("Heading: "); Serial.print(heading);
  Serial.print(" | Roll: "); Serial.print(roll);
  Serial.print(" | Pitch: "); Serial.println(pitch);

  // Prepare data to send over BLE
  char ble_message[64];
  snprintf(ble_message, sizeof(ble_message), "H:%.2f R:%.2f P:%.2f", heading, roll, pitch);

  // Send data over BLE UART
  bleuart.write(ble_message);
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  Bluefruit.ScanResponse.addName();

  // Start Advertising
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void connect_callback(uint16_t conn_handle)
{
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println("Disconnected, restarting advertisement...");
}
