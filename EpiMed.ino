#include <bluefruit.h>
#include <Wire.h>
#include <vl53l4cd_class.h>

// Pin definitions
#define XSHUT_PIN D7
#define LED_PIN LED_BUILTIN
#define RGB_RED_PIN A1
#define RGB_GREEN_PIN A2
#define RGB_BLUE_PIN A3

// Battery management pin definitions
#define GPIO_BATTERY_CHARGE_SPEED 13
#define GPIO_BATTERY_CHARGING_ENABLE 17
#define GPIO_BATTERY_READ_ENABLE 14
#define VBAT_PIN PIN_VBAT

#define NUM_SAMPLES 10

// Battery voltage calculation constants
#define VBAT_DIVIDER_R1   1000000
#define VBAT_DIVIDER_R2   510000
#define VBAT_DIVIDER      ((float)VBAT_DIVIDER_R2 / (VBAT_DIVIDER_R1 + VBAT_DIVIDER_R2))
#define ADC_RESOLUTION    (1 << 12)
#define VREF_MV           3600
#define VBAT_CALIBRATION  0.3806F

// LiPo battery constants
#define LIPO_MIN_V 3.4
#define LIPO_MAX_V 4.2

VL53L4CD sensor_vl53l4cd_sat(&Wire, XSHUT_PIN);

// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

void setup() {
  Serial.begin(115200);

  Serial.println("EpiPen BLE UART Example");
  Serial.println("------------------------\n");

  // Setup the BLE LED to be enabled on CONNECT
  Bluefruit.autoConnLed(true);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("EpiPen");
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("EpiMed");
  bledis.setModel("V1");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();

  // Set up battery management pins
  pinMode(GPIO_BATTERY_CHARGE_SPEED, OUTPUT);
  pinMode(GPIO_BATTERY_CHARGING_ENABLE, INPUT);
  pinMode(GPIO_BATTERY_READ_ENABLE, OUTPUT);

  // Enable fast charging and battery reading
  setFastCharging(true);
  enableBatteryReading(true);

  Wire.begin();
  sensor_vl53l4cd_sat.begin();
  sensor_vl53l4cd_sat.VL53L4CD_Off();
  sensor_vl53l4cd_sat.InitSensor();
  sensor_vl53l4cd_sat.VL53L4CD_SetRangeTiming(200, 0);
  sensor_vl53l4cd_sat.VL53L4CD_StartRanging();

  pinMode(LED_PIN, OUTPUT);
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);

  setRGBColor(0, 0, 0);

  analogReference(AR_INTERNAL);
  analogReadResolution(12);

  checkChargingStatus();
  readVoltage();

  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
}

void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

float readVBAT(void) {
  uint32_t total_raw = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    total_raw += analogRead(VBAT_PIN);
    delay(10);
  }
  int adcReading = total_raw / (float)NUM_SAMPLES;
  float adcVoltage = (adcReading * VREF_MV) / ADC_RESOLUTION;
  float batteryVoltage = (adcVoltage / VBAT_DIVIDER) * VBAT_CALIBRATION;
  return batteryVoltage;
}

void setFastCharging(bool enable) {
  digitalWrite(GPIO_BATTERY_CHARGE_SPEED, enable ? HIGH : LOW);
}

void enableBatteryReading(bool enable) {
  digitalWrite(GPIO_BATTERY_READ_ENABLE, enable ? HIGH : LOW);
}

bool isCharging() {
  return digitalRead(GPIO_BATTERY_CHARGING_ENABLE) == HIGH;
}

void checkChargingStatus() {
  bool charging = isCharging();
  bleuart.print("Charging status: ");
  bleuart.println(charging ? "Charging" : "Not charging");
}

void readVoltage() {
  float vbat_v = readVBAT() / 1000.0;
  float vbat_per = lipoToPercent(vbat_v);
  // bleuart.print("Voltage: ");
  // bleuart.print(vbat_v, 2);
  // bleuart.print("V (");
  bleuart.print(vbat_per, 2);
  // bleuart.println("%)");
  // blebas.write(vbat_per);
}

float lipoToPercent(float voltage) {
  float percentage = (voltage - LIPO_MIN_V) / (LIPO_MAX_V - LIPO_MIN_V) * 100.0;
  return constrain(percentage, 0, 100);
}

void setRGBColor(int red, int green, int blue) {
  analogWrite(RGB_RED_PIN, red);
  analogWrite(RGB_GREEN_PIN, green);
  analogWrite(RGB_BLUE_PIN, blue);
}

void readDistance() {
  VL53L4CD_Result_t results;
  uint8_t status = sensor_vl53l4cd_sat.VL53L4CD_GetResult(&results);
  
  if (!status) {
    bleuart.print("Distance: ");
    bleuart.print(results.distance_mm);
    bleuart.println(" mm");
  } else {
    bleuart.println("Error reading distance");
  }
}

void handleCommand(char command) {
  switch(command) {
    case '1':
      readVoltage();
      break;
    case '2':
      readDistance();
      break;
    case '3':
      checkChargingStatus();
      break;
    case '4':
      setFastCharging(true);
      bleuart.println("Fast charging enabled");
      break;
    case '5':
      setFastCharging(false);
      bleuart.println("Slow charging enabled");
      break;
    default:
      bleuart.println("Unknown command");
  }
}

void loop() {
  // Forward from BLEUART to HW Serial
  while (bleuart.available()) {
    char ch = (char) bleuart.read();
    if (ch >= '1' && ch <= '9') {  // Check if it's a valid command
      handleCommand(ch);
    } else if (ch != '\n' && ch != '\r') {  // Ignore newline and carriage return
      bleuart.println("Unknown command");
    }
  }

  sensor_vl53l4cd_sat.VL53L4CD_ClearInterrupt();
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle) {
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  (void) reason;

  Serial.println("Disconnected");
}