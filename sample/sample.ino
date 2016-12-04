#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR 0x70
#define numSensors 2
#define numCoordinates 3



float data[2][3];

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (1000)

Adafruit_BNO055 bno1 = Adafruit_BNO055(55);
Adafruit_BNO055 bno2 = Adafruit_BNO055(54);

void printData(float data[2][3])
{
  int i, j;

  Serial.print("\nCoordinates are");
  for (i = 0; i < numSensors; i++)
  {
    for (j = 0; j < numCoordinates; j++)
    {
      Serial.print(" ");
      Serial.print(data[i][j]);
    }
    Serial.println();
  }
}

float getCoordinates(int coor, sensors_event_t event)
{
  float coordinates = 0.0;

  switch (coor)
  {
    case 0:
      coordinates = (float)event.orientation.x;
      break;
    case 1:
      coordinates = (float)event.orientation.y;
      break;
    case 2:
      coordinates = (float)event.orientation.z;
      break;
  }
  return coordinates;
}

void collectData(sensors_event_t event)
{
  int i, j;
  float data[2][3];

  for (i = 0; i < numSensors; i++)
  {
    for (j = 0; j < numCoordinates; j++)
    {
      data[i][j] = getCoordinates(j, event);
    }
  }

  printData(data);
}

void displaySensorDetails(Adafruit_BNO055*bno)
{
  sensor_t sensor;
  bno->getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}


// standard Arduino setup()
void setup()
{
  while (!Serial);
  delay(1000);

  Wire.begin();

  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the 1st sensor */
  tcaselect(0);
  if (!bno1.begin())
  {
    /* There was a problem detecting the BNO05 ... check your connections */
    Serial.println("Ooops, no BNO05... Check your wiring!");
    while (1);
  }

  tcaselect(7);
  if (!bno2.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no BNO05 detected ... Check your wiring!");
    while (1);
  }

}

void loop()
{
  /* Get a new sensor event */
  sensors_event_t event;

  tcaselect(0);
  bno1.getEvent(&event);

  /* The processing sketch expects data as roll, pitch, heading */
  Serial.print(F("\nOrientation 1: "));
  Serial.print((float)event.orientation.x);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.z);
  Serial.println(F(""));
  collectData(event);

  delay(BNO055_SAMPLERATE_DELAY_MS);
  tcaselect(7);
  bno2.getEvent(&event);


  //  The processing sketch expects data as roll, pitch, heading
  Serial.print(F("\nOrientation 2: "));
  Serial.print((float)event.orientation.x);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.z);
  Serial.println(F(""));
  collectData(event);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
