#include <Wire.h>

// Structure to hold a device address and its friendly name.
struct I2CDevice {
  uint8_t address;
  const char* name;
};

// Lookup table based on Adafruit's I2C address list.
// This table includes common temperature sensors, BMP/BME sensors, and the BMP085.
const I2CDevice knownDevices[] = {
  // Temperature Sensors:
  { 0x18, "MCP9808 Temperature Sensor" },
  { 0x40, "HTU21D/Si7021 Temp/Humidity Sensor" },
  { 0x41, "Alternate HTU21D/Si7021 Temp/Humidity Sensor" },
  { 0x48, "TMP102/LM75 Temperature Sensor (or ADS1115 ADC)" },
  { 0x49, "TMP102/LM75 Temperature Sensor (or ADS1115 ADC)" },
  { 0x4A, "TMP102/LM75 Temperature Sensor (or ADS1115 ADC)" },
  { 0x4B, "TMP102/LM75 Temperature Sensor (or ADS1115 ADC)" },
  { 0x5A, "MLX90614 Infrared Temperature Sensor" },
  { 0x5B, "MLX90614 Infrared Temperature Sensor (Alternate)" },

  // Barometric and Environmental Sensors:
  { 0x76, "BMP280/BME280 Barometric Pressure Sensor" },
  { 0x77, "BMP085/BMP180/BMP280/BME280 Barometric Pressure Sensor" },

  // Other Devices:
  { 0x27, "PCF8574 I/O Expander (commonly for I2C LCD)" },
  { 0x3C, "SSD1306 OLED Display" },
  { 0x68, "MPU6050 Gyro/Accelerometer or DS1307 RTC" },
  { 0x69, "Alternate for MPU6050 Gyro/Accelerometer" }
};
const int knownDevicesCount = sizeof(knownDevices) / sizeof(knownDevices[0]);

// Function to scan the I2C bus and print device details.
void scanI2CBus() {
  Serial.println("\nScanning I2C bus...");
  byte error, address;
  int devicesFound = 0;

  // Loop through all possible I2C addresses (1-126)
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {  // device responded at this address
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.print(" -> ");

      // Check if the found device matches any in our lookup table.
      bool known = false;
      for (int i = 0; i < knownDevicesCount; i++) {
        if (knownDevices[i].address == address) {
          Serial.println(knownDevices[i].name);
          known = true;
          break;
        }
      }
      if (!known) {
        Serial.println("Unknown device");
      }
      devicesFound++;
      delay(10);  // Small delay for stability
    }
  }

  // Print summary
  if (devicesFound == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.print("Scan complete, ");
    Serial.print(devicesFound);
    Serial.println(" device(s) found.");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;  // Wait for Serial port to initialize.
  }

  
  /*
  int resetPin = 16;
  Serial.print("Toggling OLED reset pin: ");
  Serial.println(resetPin);
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, LOW);
  delay(50);  // Hold low for 50ms
  digitalWrite(resetPin, HIGH);
  delay(50);  // Wait 50ms after releasing
*/

  Serial.println("\nI2C Scanner with Device Lookup encapsulated in a function");
  //Wire.begin(4, 15);
  Wire.begin();
}

void loop() {
  scanI2CBus();
  delay(5000);  // Wait 5 seconds before the next scan
}
