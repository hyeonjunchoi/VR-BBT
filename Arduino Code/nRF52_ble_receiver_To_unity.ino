#include <bluefruit.h>

// BLE Service
BLEUart bleuart; // UART over BLE

// BLE device name
const char* device_name = "Receiver_Device";

void setup(void) {
  Serial.begin(115200);

  // BLE Initialization
  Serial.println("Initializing BLE...");
  Bluefruit.begin();
  Bluefruit.setName(device_name);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Start BLE UART Service
  bleuart.begin();
  bleuart.setRxCallback(bleuart_rx_callback);

  // Set up and start advertising
  startAdv();

  Serial.println("Waiting for incoming BLE data...");
}

void loop(void) {
  // No tasks needed in the main loop
}

void startAdv(void) {
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

void connect_callback(uint16_t conn_handle) {
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  (void) reason;

  Serial.println("Disconnected, restarting advertisement...");
}

void bleuart_rx_callback(uint16_t conn_handle) {
  (void) conn_handle;

  // Buffer to hold the received data
  char received_data[64];
  int received_length = bleuart.read(received_data, sizeof(received_data) - 1);
  received_data[received_length] = '\0'; // Null-terminate the string

  // Print the received data to Serial
  Serial.print("Received data: ");
  Serial.println(received_data);

  // Forward received data to Serial
  Serial.write(received_data);
}
