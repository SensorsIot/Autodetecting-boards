#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BME680.h>
#include <Adafruit_MCP9808.h>

// Create sensor objects
Adafruit_BMP280 bmp280;    // BMP280 object
Adafruit_BME280 bme280;    // BME280 object
Adafruit_BME680 bme680;    // BME680 object
Adafruit_MCP9808 mcp9808;  // MCP9808 object

// An enum to keep track of which sensor was found
enum SensorType {
  SENSOR_NONE,
  SENSOR_BMP280,
  SENSOR_BME280,
  SENSOR_BME680,
  SENSOR_MCP9808
};

// We'll store which sensor we found (if any)
SensorType foundSensor = SENSOR_NONE;

// Function to detect the sensor
SensorType detectSensor() {
  if (bmp280.begin(0x76) || bmp280.begin(0x77)) {
    Serial.println("BMP280 sensor detected");
    return SENSOR_BMP280;
  }
  else if (bme280.begin(0x76) || bme280.begin(0x77)) {
    Serial.println("BME280 sensor detected");
    return SENSOR_BME280;
  }
  else if (bme680.begin(0x76) || bme680.begin(0x77)) {
    Serial.println("BME680 sensor detected");
    return SENSOR_BME680;
  }
  else if (mcp9808.begin(0x18)) {
    Serial.println("MCP9808 sensor detected");
    return SENSOR_MCP9808;
  }
  else {
    Serial.println("No known sensor found.");
    return SENSOR_NONE;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);  // Give the bus some time to settle

  Serial.println("Starting sensor detection...");
  foundSensor = detectSensor();
  Serial.println("Sensor detection finished.");
}

void loop() {
  if (foundSensor == SENSOR_BMP280) {
    float temperature = bmp280.readTemperature();
    Serial.print("BMP280 Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
  }
  else if (foundSensor == SENSOR_BME280) {
    float temperature = bme280.readTemperature();
    Serial.print("BME280 Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
  }
  else if (foundSensor == SENSOR_BME680) {
    // BME680 typically needs a "performReading()" call to update data
    if (bme680.performReading()) {
      float temperature = bme680.temperature;
      Serial.print("BME680 Temperature: ");
      Serial.print(temperature);
      Serial.println(" °C");
    } else {
      Serial.println("BME680: Failed to perform reading");
    }
  }
  else if (foundSensor == SENSOR_MCP9808) {
    float temperature = mcp9808.readTempC();
    Serial.print("MCP9808 Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
  }
  else {
    // No sensor found, do nothing or print a message
    Serial.println("No sensor to read from.");
  }

  delay(2000); // Wait 2 seconds between reads
}