#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <vl53l4cx_class.h>

// I2C Bus 1 pins (for Sensor 1)
#define SDA_PIN_1 19
#define SCL_PIN_1 20

// I2C Bus 2 pins (for Sensors 2 and 3 - shared bus)
#define SDA_PIN_2 21
#define SCL_PIN_2 47

#define RGB_PIN 38
#define NUM_PIXELS 1

// XSHUT pins for Sensors 2 and 3 (on shared Bus 2)
#define XSHUT_PIN_2 37
#define XSHUT_PIN_3 4

#define SENSOR1_ADDRESS 0x29 
#define SENSOR2_ADDRESS 0x29  // Initial address, will be changed via XSHUT
#define SENSOR3_ADDRESS 0x2A  // Different address on same bus

Adafruit_NeoPixel pixel(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

// Create two I2C buses
TwoWire I2CBus1 = TwoWire(0);  // Bus 1 for Sensor 1
TwoWire I2CBus2 = TwoWire(1);  // Bus 2 for Sensors 2 and 3 (shared)

VL53L4CX sensor1(&I2CBus1, -1);  // -1 means no XSHUT pin
VL53L4CX_MultiRangingData_t rangingData1;
bool sensor1_ready = false;

VL53L4CX sensor2(&I2CBus2, XSHUT_PIN_2);
VL53L4CX_MultiRangingData_t rangingData2;
bool sensor2_ready = false;

VL53L4CX sensor3(&I2CBus2, XSHUT_PIN_3);
VL53L4CX_MultiRangingData_t rangingData3;
bool sensor3_ready = false;

int16_t left_distance = -1;
int16_t center_distance = -1;
int16_t right_distance = -1;
bool left_has_value = false;
bool center_has_value = false;
bool right_has_value = false;

uint8_t scanI2CBus(TwoWire &bus, const char *busName)
{
  Serial.print("\nScanning ");
  Serial.print(busName);
  Serial.println("...");
  
  uint8_t foundAddress = 0;
  uint8_t deviceCount = 0;
  
  for (uint8_t address = 1; address < 127; address++)
  {
    bus.beginTransmission(address);
    uint8_t error = bus.endTransmission();
    
    if (error == 0)
    {
      Serial.print("  I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;
      foundAddress = address;
    }
    else if (error == 4)
    {
      Serial.print("  Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  if (deviceCount == 0)
  {
    Serial.println("  No I2C devices found");
  }
  else
  {
    Serial.print("  Found ");
    Serial.print(deviceCount);
    Serial.println(" device(s)");
  }
  
  return foundAddress;
}

bool initSensor(VL53L4CX &sensor, uint8_t address, uint8_t xshutPin, const char *sensorName)
{
  Serial.print("\nInitializing ");
  Serial.print(sensorName);
  Serial.println("...");

  // Enable this sensor by setting XSHUT pin HIGH (if XSHUT pin is used)
  if (xshutPin != 255)  // 255 means no XSHUT pin
  {
    pinMode(xshutPin, OUTPUT);
    digitalWrite(xshutPin, LOW);
    delay(10);
    digitalWrite(xshutPin, HIGH);
    delay(10);
  }

  VL53L4CX_Error status = sensor.begin();
  if (status != VL53L4CX_ERROR_NONE)
  {
    Serial.print(sensorName);
    Serial.print(" begin failed: ");
    Serial.println(status);
    return false;
  }

  status = sensor.InitSensor(address);
  if (status != VL53L4CX_ERROR_NONE)
  {
    Serial.print(sensorName);
    Serial.print(" init failed: ");
    Serial.println(status);
    return false;
  }

  status = sensor.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_MEDIUM);
  if (status != VL53L4CX_ERROR_NONE)
  {
    Serial.print(sensorName);
    Serial.print(" distance mode failed: ");
    Serial.println(status);
    return false;
  }

  status = sensor.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(50000);
  if (status != VL53L4CX_ERROR_NONE)
  {
    Serial.print(sensorName);
    Serial.print(" timing budget failed: ");
    Serial.println(status);
    return false;
  }

  status = sensor.VL53L4CX_StartMeasurement();
  if (status != VL53L4CX_ERROR_NONE)
  {
    Serial.print(sensorName);
    Serial.print(" start measurement failed: ");
    Serial.println(status);
    return false;
  }

  Serial.print(sensorName);
  Serial.println(" initialized successfully!");
  return true;
}

void readSensor(VL53L4CX &sensor, VL53L4CX_MultiRangingData_t &rangingData,
                bool sensor_ready, int16_t &distance, bool &has_value, const char *sensorName)
{
  if (!sensor_ready)
  {
    return;
  }



  uint8_t dataReady = 0;
  VL53L4CX_Error status = sensor.VL53L4CX_GetMeasurementDataReady(&dataReady);

  if (status == VL53L4CX_ERROR_NONE && dataReady)
  {
    status = sensor.VL53L4CX_GetMultiRangingData(&rangingData);

    if (status == VL53L4CX_ERROR_NONE)
    {
      if (rangingData.NumberOfObjectsFound > 0)
      {
        uint8_t rangeStatus = rangingData.RangeData[0].RangeStatus;
        if (rangeStatus == 0)
        {
          distance = rangingData.RangeData[0].RangeMilliMeter;
          has_value = true;
        }
      }

      sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("Initializing I2C buses");
  I2CBus1.begin(SDA_PIN_1, SCL_PIN_1);
  delay(50);
  I2CBus2.begin(SDA_PIN_2, SCL_PIN_2);
  delay(50);
  Serial.println("I2C buses initialized");

  // Initialize XSHUT pins for sensors 2 and 3
  pinMode(XSHUT_PIN_2, OUTPUT);
  pinMode(XSHUT_PIN_3, OUTPUT);
  digitalWrite(XSHUT_PIN_2, LOW);
  digitalWrite(XSHUT_PIN_3, LOW);
  delay(10);

  delay(100);  // Give buses time to settle

  // Scan Bus 1 (Sensor 1) - just to verify it's there
  Serial.println("\nScanning Bus 1 for Sensor 1...");
  scanI2CBus(I2CBus1, "Bus 1");

  pixel.begin();
  pixel.setPixelColor(0, pixel.Color(0, 0, 255));
  pixel.setBrightness(5);
  pixel.show();

  // Initialize sensors with fixed addresses
  // Sensor 1 on Bus 1 (no XSHUT needed, uses default address)
  Serial.print("\nInitializing Sensor 1 at address 0x");
  Serial.println(SENSOR1_ADDRESS, HEX);
  sensor1_ready = initSensor(sensor1, SENSOR1_ADDRESS, 255, "Sensor 1");
  
  // Initialize Sensor 2 on Bus 2
  // Enable only Sensor 2 via XSHUT, initialize it to address 0x29
  digitalWrite(XSHUT_PIN_2, HIGH);
  digitalWrite(XSHUT_PIN_3, LOW);
  delay(10);
  Serial.print("\nInitializing Sensor 2 at address 0x");
  Serial.println(SENSOR2_ADDRESS, HEX);
  sensor2_ready = initSensor(sensor2, SENSOR2_ADDRESS, XSHUT_PIN_2, "Sensor 2");
  
  // Initialize Sensor 3 on Bus 2
  // Enable Sensor 3 via XSHUT, initialize it to address 0x2A (Sensor 2 stays enabled)
  digitalWrite(XSHUT_PIN_3, HIGH);
  delay(10);
  Serial.print("\nInitializing Sensor 3 at address 0x");
  Serial.println(SENSOR3_ADDRESS, HEX);
  sensor3_ready = initSensor(sensor3, SENSOR3_ADDRESS, XSHUT_PIN_3, "Sensor 3");
  
  // Verify addresses after initialization
  delay(100);
  Serial.println("\nVerifying addresses after initialization...");
  Serial.println("Scanning Bus 2 (both sensors enabled):");
  scanI2CBus(I2CBus2, "Bus 2");
  
  // Both sensors 2 and 3 are now enabled and will work simultaneously

  if (sensor1_ready && sensor2_ready && sensor3_ready)
  {
    pixel.setPixelColor(0, pixel.Color(0, 255, 0));
    pixel.show();
  }

  Serial.println("\nStarting measurements...\n");
}

void loop()
{
  readSensor(sensor2, rangingData2, sensor2_ready, left_distance, left_has_value, "Sensor 2");
  readSensor(sensor3, rangingData3, sensor3_ready, center_distance, center_has_value, "Sensor 3");
  readSensor(sensor1, rangingData1, sensor1_ready, right_distance, right_has_value, "Sensor 1");

  if (left_has_value && center_has_value && right_has_value)
  {
    Serial.print(left_distance);
    Serial.print(", ");
    Serial.print(center_distance);
    Serial.print(", ");
    Serial.println(right_distance);
  }

  delay(10);
}
