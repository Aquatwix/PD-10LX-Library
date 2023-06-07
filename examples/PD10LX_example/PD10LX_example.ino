#include <Wire.h>
#include "PD10LX.h"

PD10LX Sensor(0x40);

void setup() 
{
  Wire.begin();          // Init the I2C library
  Serial.begin(9600);    // Init serial communication
  Sensor.init();         // Init the sensor
  delay(2000);           // Waiting the serial communication
}

void loop() 
{
  Sensor.read();

  Serial.print("Pressure: "); 
  Serial.print(Sensor.getPressure(), 5); 
  Serial.println(" bar.");
  
  Serial.print("Temperature: "); 
  Serial.print(Sensor.getTemperature(), 5); 
  Serial.println(" °C");

  // Sensor.debug();
  delay(1000);
}