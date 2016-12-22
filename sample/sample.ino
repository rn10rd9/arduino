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



float data[3];

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (3000)

Adafruit_BNO055 bno1 = Adafruit_BNO055(55);
Adafruit_BNO055 bno2 = Adafruit_BNO055(54);

void printData(float d[3])
{
  int i, j;

  Serial.print("\nCoordinates are");
  for (j = 0; j < numCoordinates; j++)
  {
    Serial.print(" ");
    Serial.print(d[j]);
  }
  Serial.println();
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
  float data[3];

  for (j = 0; j < numCoordinates; j++)
  {
    data[j] = getCoordinates(j, event);
  }
  printData(data);
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

  // Quaternion data
  imu::Quaternion quat = bno1.getQuat();



  /* The processing sketch expects data as roll, pitch, heading */
  Serial.print(F("\nOrientation 1: "));
  // Quaternion data
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");



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
  //collectData(event);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
