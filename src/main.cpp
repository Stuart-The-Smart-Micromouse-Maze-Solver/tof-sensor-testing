#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>

// I2C Bus 0 pins (Wire) - sensor 1 and 2
#define SDA_PIN_0 40
#define SCL_PIN_0 41

// I2C Bus 1 pins (Wire1) - sensor 3
#define SDA_PIN_1 19
#define SCL_PIN_1 20

#define XSHUT_PIN_SENSOR_1 37
#define XSHUT_PIN_SENSOR_2 36
#define XSHUT_PIN_SENSOR_3 35

#define SENSOR1_ADDRESS 0x29
#define SENSOR2_ADDRESS 0x2A
#define SENSOR3_ADDRESS 0x29

TwoWire I2CBus0 = TwoWire(0);
TwoWire I2CBus1 = TwoWire(1);

VL53L4CX sensor1(&I2CBus0, XSHUT_PIN_SENSOR_1);
VL53L4CX sensor2(&I2CBus0, XSHUT_PIN_SENSOR_2);
VL53L4CX sensor3(&I2CBus1, XSHUT_PIN_SENSOR_3);

VL53L4CX_MultiRangingData_t rangingData1;
VL53L4CX_MultiRangingData_t rangingData2;
VL53L4CX_MultiRangingData_t rangingData3;

bool sensor1_ready = false;
bool sensor2_ready = false;
bool sensor3_ready = false;

// Distance values (Sensor 1=Left, Sensor 2=Center, Sensor 3=Right)
int16_t left_distance = -1;
int16_t center_distance = -1;
int16_t right_distance = -1;

bool initSensor(VL53L4CX &sensor, uint8_t address, const char *sensorName)
{
  VL53L4CX_Error status = sensor.begin();
  if (status != VL53L4CX_ERROR_NONE)
  {
    Serial.print("  ");
    Serial.print(sensorName);
    Serial.print(" begin failed: ");
    Serial.println(status);
    return false;
  }

  status = sensor.InitSensor(address);
  if (status != VL53L4CX_ERROR_NONE)
  {
    Serial.print("  ");
    Serial.print(sensorName);
    Serial.print(" init failed: ");
    Serial.println(status);
    return false;
  }

  status = sensor.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_MEDIUM);
  if (status != VL53L4CX_ERROR_NONE)
  {
    Serial.print("  ");
    Serial.print(sensorName);
    Serial.print(" distance mode failed: ");
    Serial.println(status);
    return false;
  }

  status = sensor.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(50000);
  if (status != VL53L4CX_ERROR_NONE)
  {
    Serial.print("  ");
    Serial.print(sensorName);
    Serial.print(" timing budget failed: ");
    Serial.println(status);
    return false;
  }

  status = sensor.VL53L4CX_StartMeasurement();
  if (status != VL53L4CX_ERROR_NONE)
  {
    Serial.print("  ");
    Serial.print(sensorName);
    Serial.print(" start measurement failed: ");
    Serial.println(status);
    return false;
  }

  Serial.print("  ");
  Serial.print(sensorName);
  Serial.print(" initialized successfully at address 0x");
  Serial.println(address, HEX);
  return true;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("Initializing I2C buses...");
  I2CBus0.begin(SDA_PIN_0, SCL_PIN_0);
  I2CBus1.begin(SDA_PIN_1, SCL_PIN_1);
  Serial.println("I2C buses initialized");

  pinMode(XSHUT_PIN_SENSOR_1, OUTPUT);
  pinMode(XSHUT_PIN_SENSOR_2, OUTPUT);
  pinMode(XSHUT_PIN_SENSOR_3, OUTPUT);
  digitalWrite(XSHUT_PIN_SENSOR_1, LOW);
  digitalWrite(XSHUT_PIN_SENSOR_2, LOW);
  digitalWrite(XSHUT_PIN_SENSOR_3, LOW);
  delay(10);

  Serial.println("\nInitializing Sensor 1 on Bus 0...");
  digitalWrite(XSHUT_PIN_SENSOR_1, HIGH);
  digitalWrite(XSHUT_PIN_SENSOR_2, LOW);
  delay(100);
  sensor1_ready = initSensor(sensor1, SENSOR1_ADDRESS, "Sensor 1");

  Serial.println("\nInitializing Sensor 2 on Bus 0...");
  digitalWrite(XSHUT_PIN_SENSOR_2, HIGH);
  delay(100);
  sensor2_ready = initSensor(sensor2, SENSOR2_ADDRESS, "Sensor 2");

  Serial.println("\nInitializing Sensor 3 on Bus 1...");
  digitalWrite(XSHUT_PIN_SENSOR_3, HIGH);
  delay(100);
  sensor3_ready = initSensor(sensor3, SENSOR3_ADDRESS, "Sensor 3");

  Serial.print("Sensor 1 (Bus 0, 0x29): ");
  Serial.println(sensor1_ready ? "READY" : "FAILED");
  Serial.print("Sensor 2 (Bus 0, 0x2A): ");
  Serial.println(sensor2_ready ? "READY" : "FAILED");
  Serial.print("Sensor 3 (Bus 1, 0x29): ");
  Serial.println(sensor3_ready ? "READY" : "FAILED");
}

bool readSensor(VL53L4CX &sensor, VL53L4CX_MultiRangingData_t &rangingData, int16_t &distance)
{
  uint8_t dataReady = 0;
  VL53L4CX_Error status = sensor.VL53L4CX_GetMeasurementDataReady(&dataReady);

  if (status == VL53L4CX_ERROR_NONE && dataReady)
  {
    status = sensor.VL53L4CX_GetMultiRangingData(&rangingData);

    if (status == VL53L4CX_ERROR_NONE)
    {
      sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
      if (rangingData.NumberOfObjectsFound > 0)
      {
        distance = rangingData.RangeData[0].RangeMilliMeter;
        return true;
      }
    }
  }
  return false;
}

void loop()
{
  // Sensor 1 - Left
  if (sensor1_ready)
  {
    readSensor(sensor1, rangingData1, left_distance);
  }
  // Sensor 2 - Center
  if (sensor2_ready)
  {
    readSensor(sensor2, rangingData2, center_distance);
  }
  // Sensor 3 - Right
  if (sensor3_ready)
  {
    readSensor(sensor3, rangingData3, right_distance);
  }

  Serial.print(left_distance);
  Serial.print(", ");
  Serial.print(center_distance);
  Serial.print(", ");
  Serial.println(right_distance);

  delay(10);
}
