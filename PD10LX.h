#ifndef PD10LX_H
#define PD10LX_H

#include "Arduino.h"

class PD10LX 
{
  public:

    // Constructor for PD10LX class, takes the slave address as a parameter
    PD10LX(uint8_t slave_address);

    /** 
    * Initializes the registers of the PD10LX sensor
    */
    bool init();

    /** 
    * Get the calibration date of the PD10LX sensor
    */
    bool sensorCalibration();

    /** 
    * Get the information values of the PD10LX sensor
    */
    bool sensorInformation();

    /** 
    * Check the information about active channels of the PD10LX sensor
    */
    bool sensorActiveChannels();

    /**
    * Reads the onboard memory map to determine temperature and pressure
    */
    void read();

    /**
    * Returns the pressure in bar
    * @return Pressure value in bar
    */
    float getPressure();

    /**
    * Returns the temperature in degrees Celsius
    * @return Temperature value in degrees Celsius
    */
    float getTemperature();

    /** 
    * Displays the contents of the class's registers and variables
    */
    void debug();

    /**
    * Converts a hexadecimal value to IEEE 754 floating-point representation
    * @param hex_value Hexadecimal value to be converted
    * @return Floating-point value
    */
    float ieee754_converter(uint32_t hex_value);

    /**
    * Calculates the CRC8 value for the given data buffer
    * @param Buffer Pointer to the input data buffer
    * @param Buffersize Size of the input data buffer
    * @return Calculated CRC8 value
    */
    uint8_t Crc8(uint8_t* Buffer, uint8_t Buffersize);

  private:
    
    // Flag indicating if the PD10LX sensor has been initialized
    bool flag_init = false;

    // Flag indicating if data has been read from the sensor
    bool flag_data = false;

    // Slave address of the PD10LX sensor
    uint8_t slave_address;

    // State register value of the PD10LX sensor
    uint8_t state_register;

    // StatePT register value of the PD10LX sensor
    uint8_t statePT_register;

    // Amount of bytes for data transmission
    uint8_t byte_amount;

    // Block number for data transmission
    uint8_t block_nr;

    // Combination of byte_amount and block_nr for ACK transmission
    uint8_t ackBlockByteAmt;

    // Address used for communication with the PD10LX sensor
    uint8_t address;

    // CRC (Cyclic Redundancy Check) value for data integrity
    uint8_t CRC;

    // Raw pressure value obtained from the PD10LX sensor
    uint32_t int_pressure;

    // Raw temperature value obtained from the PD10LX sensor
    uint32_t int_temperature;

    // Pressure value calculated from the raw pressure value
    float pressure;

    // Temperature value calculated from the raw temperature value
    float temperature;

    // Calibration day of the sensor
    int calibration_day;

    // Calibration month of the sensor
    int calibration_month;

    // Calibration year of the sensor
    int calibration_year;

    // Minimum value that the sensor can reach 
    float P_min;

    // Maximal value that the sensor can reach
    float P_max;

    // Actives channels
    uint8_t channels;
};

#endif