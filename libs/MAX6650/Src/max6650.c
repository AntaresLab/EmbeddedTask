/*
 * max6650.c
 *
 *  Created on: 24 окт. 2021 г.
 *      Author: Antares
 */

#include "max6650.h"
#include <string.h>

static bool     MAX6650_isInitialized = false;
static uint16_t MAX6650_i2cAddress;
static uint16_t MAX6650_minRPM;
static uint16_t MAX6650_maxRPM;

// Converts fan speed in % to the MAX6650 speed register value
static uint8_t valueToSpeed(uint8_t value){
    uint16_t rpm = (((uint32_t) MAX6650_maxRPM - MAX6650_minRPM) * (value - 1) /
                    (MAX6650_MAX_FAN_VALUE - 1)) + MAX6650_minRPM;
    uint8_t speed = ((uint16_t) MAX6650_MAX_FAN_RPM * 2 / rpm) - 1;
    return speed;
}

// Converts the MAX6650 speed register value to the fan speed in RPM
static uint16_t speedToRpm(uint8_t speed){
    return 60000 / ((uint16_t) speed + 1);
}

// Converts fan speed in RPM to the speed in %
static uint8_t rpmToValue(uint16_t currentRPM){
    return (((uint32_t) currentRPM - MAX6650_minRPM) * (MAX6650_MAX_FAN_VALUE - 1) /
            (MAX6650_maxRPM - MAX6650_minRPM)) + 1;
}

bool MAX6650_init(MAX6650_Address address,
                  MAX6650_I2CSpeed i2cSpeed,
                  MAX6650_FanVoltage fanVoltage,
                  uint16_t minRPM,
                  uint16_t maxRPM){

    // Check if I2C address is valid and set for the next purposes
    if(!IS_VALID_MAX6650_I2C_ADDRESS(address) ||
            (maxRPM > MAX6650_MAX_FAN_RPM) ||
            (minRPM < MAX6650_MIN_FAN_RPM) ||
            (minRPM > maxRPM)){
        return false;
    }
    MAX6650_i2cAddress = (uint16_t) address;

    // Try to set the I2C bus speed
    if(!setup_i2c(i2cSpeed == MAX6650_I2C_SPEED_400K ? true : false)){
        return false;
    }

    // Set minimal and maximal fan speed values
    MAX6650_minRPM = minRPM;
    MAX6650_maxRPM = maxRPM;

    // Set maximal fan speed
    uint8_t buffer = valueToSpeed(MAX6650_MAX_FAN_VALUE);
    if(!write_i2c(MAX6650_i2cAddress, (uint16_t) MAX6650_SPEED_REGISTER, &buffer, sizeof(buffer))){
        return false;
    }

    // Enable fan speed regulation, set fan voltage and the widest fan speed range
    if(!read_i2c(MAX6650_i2cAddress, (uint16_t) MAX6650_CONFIG_REGISTER, &buffer, sizeof(buffer))){
        return false;
    }
    buffer &= ~(uint8_t) MAX6650_OPERATING_MODE;
    buffer |=  (uint8_t) MAX6650_OPERATING_MODE_CLOSED_LOOP;
    buffer &= ~(uint8_t) MAX6650_FAN_VOLTAGE;
    buffer |=  (uint8_t)(fanVoltage == MAX6650_FAN_VOLTAGE_5V ? MAX6650_FAN_VOLTAGE_5V :
                                                                MAX6650_FAN_VOLTAGE_12V);
    buffer &= ~(uint8_t) MAX6650_SCALE;
    buffer |=  (uint8_t) MAX6650_SCALE_DIV_1;
    if(!write_i2c(MAX6650_i2cAddress, (uint16_t) MAX6650_CONFIG_REGISTER, &buffer, sizeof(buffer))){
        return false;
    }

    // If all is OK set initialization OK flag
    MAX6650_isInitialized = true;
    return true;
}

bool MAX6650_setValue(uint8_t value){

    // Check if fan driver is initialized and value is in the range
    if(!MAX6650_isInitialized || (value > MAX6650_MAX_FAN_VALUE)){
        return false;
    }

    // Read fan speed regulation mode
    uint8_t buffer;
    if(!read_i2c(MAX6650_i2cAddress, (uint16_t) MAX6650_CONFIG_REGISTER, &buffer, sizeof(buffer))){
        return false;
    }

    if(value == 0){

        // Switch fan speed regulation off
        if((buffer & MAX6650_OPERATING_MODE) != MAX6650_OPERATING_MODE_SHUTDOWN){
            buffer &= ~(uint8_t) MAX6650_OPERATING_MODE;
            buffer |=  (uint8_t) MAX6650_OPERATING_MODE_SHUTDOWN;
            if(!write_i2c(MAX6650_i2cAddress, (uint16_t) MAX6650_CONFIG_REGISTER, &buffer, sizeof(buffer))){
                return false;
            }
        }

    }else{

        // Switch fan speed regulation on
        if((buffer & MAX6650_OPERATING_MODE) != MAX6650_OPERATING_MODE_CLOSED_LOOP){
            buffer &= ~(uint8_t) MAX6650_OPERATING_MODE;
            buffer |=  (uint8_t) MAX6650_OPERATING_MODE_CLOSED_LOOP;
            if(!write_i2c(MAX6650_i2cAddress, (uint16_t) MAX6650_CONFIG_REGISTER, &buffer, sizeof(buffer))){
                return false;
            }
        }

        // Set fan driver speed register value
        buffer = valueToSpeed(value);
        if(!write_i2c(MAX6650_i2cAddress, (uint16_t) MAX6650_SPEED_REGISTER, &buffer, sizeof(buffer))){
            return false;
        }
    }

    return true;
}

MAX6650_FanParameters MAX6650_getFanParameters(){

    MAX6650_FanParameters fanParameters = {0, 0, false};

    // Check if fan driver is initialized
    if(!MAX6650_isInitialized){
        return fanParameters;
    }

    // Read fan speed regulation mode
    uint8_t buffer;
    if(!read_i2c(MAX6650_i2cAddress, (uint16_t) MAX6650_CONFIG_REGISTER, &buffer, sizeof(buffer))){
        return fanParameters;
    }

    // If fan speed regulation is switched on read speed register and calculate values
    if((buffer & MAX6650_OPERATING_MODE) == MAX6650_OPERATING_MODE_CLOSED_LOOP){
        if(!read_i2c(MAX6650_i2cAddress, (uint16_t) MAX6650_SPEED_REGISTER, &buffer, sizeof(buffer))){
            return fanParameters;
        }
        fanParameters.rpm = speedToRpm(buffer > 0 ? buffer : 1);
        fanParameters.value = rpmToValue(fanParameters.rpm);
        fanParameters.isValid = true;

    }else if((buffer & MAX6650_OPERATING_MODE) == MAX6650_OPERATING_MODE_SHUTDOWN){

        // If fan speed regulation is switched off return 0 values
        fanParameters.isValid = true;
    }
    
    return fanParameters;
}
