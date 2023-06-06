#include "PD10LX.h"
#include <Wire.h>
#include <math.h>

/**
 * @brief Constructor of the PD10LX class.
 * @param slave_address The address of the slave.
 */
PD10LX::PD10LX(uint8_t slave_address) 
{
	this->slave_address = slave_address; // Initialize the member variable slave_address with the passed argument value
}

/**
 * @brief Initializes the members of the PD10LX object.
 */
void PD10LX::init() 
{
  this->byte_amount = 0x4;                                                   // Initialize byte_amount with the hexadecimal value 0x4
  this->block_nr = 0x0;                                                      // Initialize block_nr with the hexadecimal value 0x0
  this->ackBlockByteAmt = this->byte_amount << 4 | this->block_nr;           // Left shift byte_amount by 4 bits followed by an OR operation with block_nr to obtain ackBlockByteAmt
  this->address = 0x00;                                                      // Initialize address with the hexadecimal value 0x00

  this->flag_init = true;                                                    // Set flag_init to true to indicate that initialization is complete
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
  Wire.write(this->ackBlockByteAmt);                                // Write the ackBlockByteAmt value to the slave (Byte Amount (7...3) Block Nr (2...0))
  Wire.write(this->address);                                        // Write the address to the slave
  uint8_t buffer[2] = {this->ackBlockByteAmt, this->address};
  this->CRC = this->Crc8(buffer, 2);                                // Calculate the CRC value for the buffer
  Wire.write(this->CRC);                                            // Write the CRC value to the slave
  byte err = Wire.endTransmission();                                // End the transmission and check for any errors
  while(err) {Serial.println("I2C communication error.");}

  delay(9);

  Wire.requestFrom(this->slave_address, 11);                        // Request 11 bytes of data from the slave in reading mode 
  
  if(Wire.available())
  {
    // Get State and StatePT register to get information about I2C communication
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
uint8_t PD10LX::Crc8(uint8_t* Buffer, uint8_t Buffersize) {
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