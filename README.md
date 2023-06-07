# PD-10LX-Library

Arduino library for Keller X-Line sensors. Simplifies the integration and use of PD-10LX and other X-Line sensors.

You can find the <a href="https://www.kelleramerica.com/file-cache/website_component/5e2f286f9d8b1060a188c36a/manuals/1625167817459">Description of the Communication protocol for X-Line Pressure Transmitters from KELLER</a> if you want to understand the reasoning of the code.

Here is the <a href="https://keller-druck.com/en/products/pressure-transducers/oem-differential-pressure-transducers/series-pd-10l">datasheet</a> of the sensor used during the design of the library. 

# Retrieving data from the sensor

<img src="https://cdn.discordapp.com/attachments/907403851131416587/1115935216104636507/image.png" alt="Console" width="250" height="120"/> 

Once you have imported the library into your Arduino IDE, you can load an example to retrieve the pressure and temperature data from the sensor.

# Debug mode

<img src="https://cdn.discordapp.com/attachments/907403851131416587/1115936367420129361/image.png" alt="Console" width="300" height="170"/> 

If you want to check the contents of the registers, you can do so. Use the debug() function after read().
