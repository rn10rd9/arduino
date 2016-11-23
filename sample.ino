#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR 0x70

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (1000)

Adafruit_BNO055 bno1 = Adafruit_BNO055(55);
Adafruit_BNO055 bno2 = Adafruit_BNO055(50);

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
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the 1st sensor */
 //tcaselect(3);
  if (!bno1.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
//  /* Initialise the 2nd sensor */
 // tcaselect(7);
  if (!bno2.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
//  /* Display some basic information on this sensor */
  tcaselect(3);
  displaySensorDetails(&bno1);
  tcaselect(7);
  displaySensorDetails(&bno2);
}

void loop()
{
  /* Get a new sensor event */
  sensors_event_t event;
  bno1.getEvent(&event);
            
  /* The processing sketch expects data as roll, pitch, heading */
  Serial.print(F("Orientation 1: "));
  Serial.print((float)event.orientation.x);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.z);
  Serial.println(F(""));

  /* Also send calibration data for each sensor. */
  uint8_t sys, gyro, accel, mag = 0;
  bno1.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print(F("Calibration: "));
  Serial.print(sys, DEC);
  Serial.print(F(" "));
  Serial.print(gyro, DEC);
  Serial.print(F(" "));
  Serial.print(accel, DEC);
  Serial.print(F(" "));
  Serial.println(mag, DEC);
   
  delay(BNO055_SAMPLERATE_DELAY_MS);

   bno2.getEvent(&event);


  /* The processing sketch expects data as roll, pitch, heading */
  Serial.print(F("Orientation 2: "));
  Serial.print((float)event.orientation.x);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.z);
  Serial.println(F(""));

  /* Also send calibration data for each sensor. */

  bno2.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print(F("Calibration 2: "));
  Serial.print(sys, DEC);
  Serial.print(F(" "));
  Serial.print(gyro, DEC);
  Serial.print(F(" "));
  Serial.print(accel, DEC);
  Serial.print(F(" "));
  Serial.println(mag, DEC);
   
  delay(BNO055_SAMPLERATE_DELAY_MS);
 


}
