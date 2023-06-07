#include "PD10LX.h"
#include <Wire.h>
#include <math.h>

#define MEASURE_ADR          0x00        // Address to get pressure and temperature data
#define SERIAL_NUMBER_ADR    0x30        // Address to get the serial number of the sensor
#define CALIBRATION_ADR      0x38        // Address to get the data of calibration
#define CFG_CHANNEL_ADR      0x44        // Address to see which channels are actives

#define BLOCK0_LDR           0x40        // Load value to get data from block 0 (data)
#define BLOCK3_LDR           0x43        // Load value to get data from block 3 (information)


/**
 * @brief Constructor of the PD10LX class.
 * @param slave_address The address of the slave.
 */
PD10LX::PD10LX(uint8_t slave_address) 
{
	this->slave_address = slave_address; // Initialize the member variable slave_address with the passed argument value
}


/**
  * @brief Initializes the members of the PD10LX object getting calibration date, specifications and active channels.
*/
void PD10LX::init() 
{
  this->sensorCalibration();        // Get calibration date
  this->sensorInformation();        // Get the sensor specifications
  this->sensorActiveChannels();     // Get the actives channels 

  this->flag_init = true;           // Set flag_init to true to indicate that initialization is complete                                    
}


/**
 * @brief Get information about calibration date.
 */
void PD10LX::sensorCalibration()
{
  Wire.beginTransmission(this->slave_address);                    // Start the transmission with the slave address in writing mode
  Wire.write(BLOCK3_LDR);                                         // Write the ackBlockByteAmt value to the slave (Byte Amount (7...3) Block Nr (2...0))
  Wire.write(CALIBRATION_ADR);                                    // Write the address to the slave
  uint8_t buffer[2] = {BLOCK3_LDR, CALIBRATION_ADR};
  this->CRC = this->Crc8(buffer, 2);                              // Calculate the CRC value for the buffer
  Wire.write(this->CRC);                                          // Write the CRC value to the slave
  byte err = Wire.endTransmission();  
  Serial.println(err);                                            // End the transmission and check for any errors
  while(err) {Serial.println("I2C communication error.");}
  
  delay(9);
 
  Wire.requestFrom(this->slave_address, 7); // Request data from the slave
  
  if(Wire.available())
  {
    // State and StatePT registers
    Wire.read(); Wire.read();

    this->calibration_day = Wire.read();                                                    // Read the calibration day from the slave
    this->calibration_month = Wire.read();                                                  // Read the calibration month from the slave
    this->calibration_year = int((uint16_t(Wire.read()) << 8) | uint16_t(Wire.read()));     // Read the calibration year from the slave
  }
}


/**
 * @brief Retrieves sensor information.
 */
void PD10LX::sensorInformation()
{
  // Sending data to the slave
  Wire.beginTransmission(this->slave_address);                  // Start the transmission with the slave address in writing mode
  Wire.write(BLOCK3_LDR);                                       // Write the ackBlockByteAmt value to the slave (Byte Amount (7...3) Block Nr (2...0))
  Wire.write(MEASURE_ADR);                                      // Write the address to the slave
  uint8_t buffer[2] = {BLOCK3_LDR, MEASURE_ADR};
  this->CRC = this->Crc8(buffer, 2);                            // Calculate the CRC value for the buffer
  Wire.write(this->CRC);                                        // Write the CRC value to the slave
  byte err = Wire.endTransmission();                            // End the transmission and check for any errors
  while(err) {Serial.println("I2C communication error.");}
  
  delay(9);
 
  Wire.requestFrom(this->slave_address, 43); // Request data from the slave
  
  if(Wire.available())
  {
    // State and StatePT registers
    Wire.read(); Wire.read();

    // Get hexadecimal (32 bits) value from Data register
    this->P_min = ieee754_converter((uint32_t(Wire.read()) << 24) | (uint32_t(Wire.read()) << 16) | (uint32_t(Wire.read()) << 8) | uint32_t(Wire.read())); // Get the first 16 bits
    this->P_max = ieee754_converter((uint32_t(Wire.read()) << 24) | (uint32_t(Wire.read()) << 16) | (uint32_t(Wire.read()) << 8) | uint32_t(Wire.read())); // Get the last 16 bits
  }
}


/**
 * @brief Get active sensor channels.
 */
void PD10LX::sensorActiveChannels() 
{
  // Sending data to the slave
  Wire.beginTransmission(this->slave_address);                      // Start the transmission with the slave address in writing mode
  Wire.write(BLOCK3_LDR);                                           // Write the ackBlockByteAmt value to the slave (Byte Amount (7...3) Block Nr (2...0))
  Wire.write(CFG_CHANNEL_ADR);                                      // Write the address to the slave
  uint8_t buffer[2] = {BLOCK3_LDR, CFG_CHANNEL_ADR};
  this->CRC = this->Crc8(buffer, 2);                                // Calculate the CRC value for the buffer
  Wire.write(this->CRC);                                            // Write the CRC value to the slave
  byte err = Wire.endTransmission();                                // End the transmission and check for any errors
  while(err) {Serial.println("I2C communication error.");}
  
  delay(9);
 
  Wire.requestFrom(this->slave_address, 11); // Request data from the slave
  
  if(Wire.available())
  {
    // Get State and StatePT register
    Wire.read(); Wire.read();
    this->channels = Wire.read(); // Read the active sensor channels from the slave
  }
}


/**
* @brief Reads data from the PD10LX sensor.
* @note: Make sure to initialize the sensor before calling this function.
*/
void PD10LX::read() 
{
  // Check initialization before obtaining data
  while(!this->flag_init)
  {
    Serial.println("Sensor needs to be initialized before reading.");
    delay(5000);
  }

  // Sending data to the slave
  Wire.beginTransmission(this->slave_address);                      // Start the transmission with the slave address in writing mode
  Wire.write(BLOCK0_LDR);                                           // Write the ackBlockByteAmt value to the slave (Byte Amount (7...3) Block Nr (2...0))
  Wire.write(MEASURE_ADR);                                          // Write the address to the slave
  uint8_t buffer[2] = {BLOCK0_LDR, MEASURE_ADR};
  this->CRC = this->Crc8(buffer, 2);                                // Calculate the CRC value for the buffer
  Wire.write(this->CRC);                                            // Write the CRC value to the slave
  byte err = Wire.endTransmission();                                // End the transmission and check for any errors
  while(err) {Serial.println("I2C communication error.");}

  delay(9);

  Wire.requestFrom(this->slave_address, 11);                        // Request 11 bytes of data from the slave in reading mode 

  if(Wire.available())
  {
    // Get State and StatePT register
    this->state_register = Wire.read();
    this->statePT_register = Wire.read();

    // Get hexadecimal (32 bits) value from Data register
    this->int_pressure = (uint32_t(Wire.read()) << 24) | (uint32_t(Wire.read()) << 16) | (uint32_t(Wire.read()) << 8) | uint32_t(Wire.read());        // Get the first 16 bits
    this->int_temperature = (uint32_t(Wire.read()) << 24) | (uint32_t(Wire.read()) << 16) | (uint32_t(Wire.read()) << 8) | uint32_t(Wire.read());     // Get the last 16 bits

    // Get CRC value
    this->CRC = Wire.read();
    this->flag_data = true;
  }
}


/**
 * @brief Retrieves the pressure value.
 * @note Make sure to read the data before calling this function.
 * @return The pressure value converted to float.
 */
float PD10LX::getPressure() 
{
  // Check that the data has been acquired before sending it back
  while(!this->flag_data)
  {
    Serial.println("The data must be read before it can be retrieved.");
    delay(5000);
  }

  return ieee754_converter(this->int_pressure);
}


/**
 * @brief Retrieves the temperature value.
 * @note : Make sure to read the data before calling this function.
 * @return The temperature value converted to float.
 */
float PD10LX::getTemperature() 
{
  // Check that the data has been acquired before sending it back
  while(!this->flag_data)
  {
    Serial.println("The data must be read before it can be retrieved.");
    delay(5000);
  }

  return ieee754_converter(this->int_temperature);
}


/**
 * @brief Prints debug information related to the PD10LX sensor.
 */
void PD10LX::debug() 
{
    // Print State register value
    Serial.print("State: 0x");
    Serial.println(this->state_register, HEX);

    // Print StatePT register value
    Serial.print("StatePT: 0x");
    Serial.println(this->statePT_register, HEX);

    // Print Data register values (pressure and temperature)
    Serial.print("Data: 0x");
    Serial.print(this->int_pressure, HEX);
    Serial.print(" ");
    Serial.println(this->int_temperature, HEX);

    Serial.print("Pressure: ");
    Serial.println(this->getPressure(), 5);
    Serial.print("Temperature: ");
    Serial.println(this->getTemperature(), 4);

    // Print CRC value
    Serial.print("CRC: 0x");
    Serial.println(this->CRC, HEX);

    // Print separator line
    Serial.println("—————————————————————————————————————————————");

    // Print flag_init status
    Serial.print("Flag Init: ");
    Serial.println(this->flag_init);

    // Print flag_data status
    Serial.print("Flag Data: ");
    Serial.println(this->flag_data);

    // Print date of calibration
    Serial.print("Calibration: ");
    Serial.print(this->calibration_day);
    Serial.print("/");
    Serial.print(this->calibration_month);
    Serial.print("/");
    Serial.println(this->calibration_year);

    // Print information
    Serial.print("P_max: "); 
    Serial.println(this->P_max);
    Serial.print("P_min: "); 
    Serial.println(this->P_min);
    Serial.print("CFG Channels (TOB2, TOB1, T, P2, P1): "); 
    Serial.println(this->channels, BIN);

    // Print separator line
    Serial.println("—————————————————————————————————————————————");
}


/**
 * @name ieee754_converter
 * @brief Convert a hexadecimal (32-bit) value into a decimal value using the IEEE-754 floating point converter
 * @param hex_value the hexadecimal value to convert
 * @return the conversion into decimals
*/
float PD10LX::ieee754_converter(uint32_t hex_value) 
{
    // Cast hexa value into deciaml value
    uint32_t decimal_value = hex_value;
    
    // Extraction of bits
    uint32_t sign_bit = (decimal_value >> 31) & 0x1;
    uint32_t exponent_bits = (decimal_value >> 23) & 0xFF;
    uint32_t fraction_bits = decimal_value & 0x7FFFFF;
    
    // Calculate values
    int8_t sign = (-1) * (2 * sign_bit - 1);
    int8_t exponent = exponent_bits - 127;
    float fraction = 1.0 + (float(fraction_bits) / pow(2, 23));
    
    // Calculate the final decimal value
    float decimal_result = sign * fraction * pow(2, exponent);
    
    return decimal_result;
}


/**
 * @name Crc8
 * @brief This function calculates the CRC8 value for the given buffer of data.
 * @param Buffer Pointer to the input data buffer.
 * @param Buffersize Size of the input data buffer.
 * @return The calculated CRC8 value.
 */
uint8_t PD10LX::Crc8(uint8_t* Buffer, uint8_t Buffersize) 
{
  uint16_t data;
  uint16_t crc = 0;
  uint8_t i, j;
  for (j = 0; j < Buffersize; j++) {
    data = (uint16_t)*(Buffer+j);
    crc ^= (data << 8);
    for(i = 0; i < 8; i++) {
      if (crc & 0x8000)
        crc ^= 0x8380;
      crc = crc << 1;
    }
  }
  return (uint8_t)(crc >> 8);
}