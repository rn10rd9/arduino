#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR 0x70
#define numSensors 2
#define numCoordinates 4



//float data[3];

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (500)

Adafruit_BNO055 bno1 = Adafruit_BNO055(55);
Adafruit_BNO055 bno2 = Adafruit_BNO055(54);

void sendData(float d[4])
{
  int i, j;

 // Serial.print("\nCoordinates are");
  for (j = 0; j < numCoordinates; j++)
  {
    //Serial.print(" ");
    Serial.print(d[j], 4);
  }
  //Serial.println();
}

void getCoordinates(int coor, imu::Quaternion quat)
{
  float coordinates = -55555.55;

  switch (coor)
  {
    case 0:
      //coordinates = quat.w();
      Serial.println(quat.w());
     
      break;
    case 1:
      //coordinates = quat.x();
      Serial.println(quat.x());;
      break;
    case 2:
      //coordinates = quat.y();
      Serial.println(quat.y());
      break;
    case 3:
      //coordinates = quat.z();
      Serial.println(quat.z());
      break;

      
  }
  return coordinates;
}

void collectData(imu::Quaternion quat)
{
  int i, j;
  float data[4];
 

  for (j = 0; j < numCoordinates; j++)
  {
    getCoordinates(j, quat);
  }

  //return data;
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
  //Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the 1st sensor */
  //Serial.println("HERE");
  tcaselect(0);
  //Serial.println("HERE");
  if (!bno1.begin())
  {
    /* There was a problem detecting the BNO05 ... check your connections */
    Serial.println("Ooops, no BNO05... Check your wiring!");
    while (1);
  }
  //Serial.println("HERE TOO");
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
  //Serial.println("Check your wiring!");

  // Quaternion data
  imu::Quaternion quat1 = bno1.getQuat();
  
  tcaselect(7);
  bno2.getEvent(&event);
    
  // Quaternion data
  imu::Quaternion quat2 = bno2.getQuat();

  float* data1;
  float* data2;
 

  /* The processing sketch expects data as roll, pitch, heading */
    //Serial.print(F("\nOrientation 1: "));
//    Serial.print("qW: ");
//    Serial.print(quat1.w(), 4);
//    Serial.print(" qX: ");
//   Serial.print(quat1.y(), 4);
//    Serial.print(" qY: ");
//    Serial.print(quat1.x(), 4);
//    Serial.print(" qZ: ");
//    Serial.print(quat1.z(), 4);
//    Serial.print("\t\t");
      collectData(quat1);
    
    //Serial.print("\n");
    delay(BNO055_SAMPLERATE_DELAY_MS);
    

  //  The processing sketch expects data as roll, pitch, heading
    //Serial.print(F("\nOrientation 2: "));
//    Serial.print("qW: ");
//      Serial.print(quat2.w(), 4);
//    Serial.print(" qX: ");
//      Serial.print(quat2.y(), 4);
//    Serial.print(" qY: ");
//    Serial.print(quat2.x(), 4);
//    Serial.print(" qZ: ");
//    Serial.print(quat2.z(), 4);
//    Serial.print("\t\t");
//    //collectData(event);*/
    collectData(quat2);
    //sendData(collectData(quat2));
    //Serial.print("\n");
    delay(BNO055_SAMPLERATE_DELAY_MS);
  
}
