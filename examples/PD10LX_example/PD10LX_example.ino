#include <Wire.h>
#include "PD10LX.h"

PD10LX Pressure(0x40);

void setup() 
{
  Wire.begin();          // Init the I2C library
  Serial.begin(9600);    // Init serial communication
  Pressure.init();
  delay(2000);           // Waiting the serial communication
}

void loop() 
{
  Pressure.read();

  Serial.print("Pressure: "); 
  Serial.print(Pressure.getPressure(), 5); 
  Serial.println(" bar.");
  
  Serial.print("Temperature: "); 
  Serial.print(Pressure.getTemperature(), 5); 
  Serial.println(" °C");

  Serial.println();
  delay(1000);
}