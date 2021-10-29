/*
 * max6650.h
 *
 * MAX6650 library interface
 *
 *  Created on: 24 окт. 2021 г.
 *      Author: Antares
 */

#ifndef INC_MAX6650_H_
#define INC_MAX6650_H_

#include <stdint.h>
#include <stdbool.h>

// Minimal and maximal fan speed in RPM
// Limited by the library
#define MAX6650_MAX_FAN_RPM   30000
#define MAX6650_MIN_FAN_RPM   240

// Maximal fan speed in %
// Limited by the library
#define MAX6650_MAX_FAN_VALUE 100

// MAX6650 available I2C addresses
typedef enum{
    MAX6650_ADDR_HIGH_Z = 0x36,
    MAX6650_ADDR_GND_10K = 0x3E,
    MAX6650_ADDR_GND = 0x90,
    MAX6650_ADDR_VCC = 0x96
}MAX6650_Address;

#define IS_VALID_MAX6650_I2C_ADDRESS(address) (((address) == MAX6650_ADDR_HIGH_Z) ||\
                                               ((address) == MAX6650_ADDR_GND_10K) ||\
                                               ((address) == MAX6650_ADDR_GND) ||\
                                               ((address) == MAX6650_ADDR_VCC))

// MAX6650 available I2C bus speed
typedef enum{
    MAX6650_I2C_SPEED_100K = 0,
    MAX6650_I2C_SPEED_400K
}MAX6650_I2CSpeed;

// MAX6650 registers list
typedef enum{
    MAX6650_SPEED_REGISTER = 0x00,
    MAX6650_CONFIG_REGISTER = 0x02,
    MAX6650_GPIO_DEF_REGISTER = 0x04,
    MAX6650_DAC_REGISTER = 0x06,
    MAX6650_ALARM_ENABLE_REGISTER = 0x08,
    MAX6650_ALARM_REGISTER = 0x0A,
    MAX6650_TACH0_REGISTER = 0x0C,
    MAX6650_TACH1_REGISTER = 0x0E,
    MAX6650_TACH2_REGISTER = 0x10,
    MAX6650_TACH3_REGISTER = 0x12,
    MAX6650_GPIO_STAT_REGISTER = 0x14,
    MAX6650_COUNT_REGISTER = 0x16
}MAX6650_Register;

// MAX6650 CONFIG register description
typedef enum{
    MAX6650_OPERATING_MODE = 0x30,
    MAX6650_FAN_VOLTAGE = 0x08,
    MAX6650_SCALE = 0x07
}MAX6650_Config;

// MAX6650 CONFIG register Operatig Mode section values description
typedef enum{
    MAX6650_OPERATING_MODE_FULL_ON = 0x00,
    MAX6650_OPERATING_MODE_SHUTDOWN = 0x10,
    MAX6650_OPERATING_MODE_CLOSED_LOOP = 0x20,
    MAX6650_OPERATING_MODE_OPEN_LOOP = 0x30,
    MAX6650_OPERATING_MODE_DEFAULT = MAX6650_OPERATING_MODE_FULL_ON
}MAX6650_OperatingMode;

// MAX6650 CONFIG register Fan Voltage section values description
typedef enum{
    MAX6650_FAN_VOLTAGE_5V = 0x00,
    MAX6650_FAN_VOLTAGE_12V = 0x08,
    MAX6650_FAN_VOLTAGE_DEFAULT = MAX6650_FAN_VOLTAGE_12V
}MAX6650_FanVoltage;

// MAX6650 CONFIG register Tachometer Scale section values description
typedef enum{
    MAX6650_SCALE_DIV_1 = 0x00,
    MAX6650_SCALE_DIV_2 = 0x01,
    MAX6650_SCALE_DIV_4 = 0x02,
    MAX6650_SCALE_DIV_8 = 0x03,
    MAX6650_SCALE_DIV_16 = 0x04,
    MAX6650_SCALE_DEFAULT = MAX6650_SCALE_DIV_4
}MAX6650_Scale;

/*** Functions that should be defined in the application ***/
// Switshes I2C speed
// fast_speed == false => 100kHz
// fast_speed == false => 400kHz
// Returns true if switched successfully
extern bool setup_i2c(bool fast_speed);

// Writes (size) bytes from (buf) to the I2C device with (slave) bus address starting from (reg) internal address
// Returns true if data was wrote successfully
extern bool write_i2c(uint16_t slave, uint16_t reg, uint8_t* buf, uint16_t size);

// Reads (size) bytes to (buf) from the I2C device with (slave) bus address starting from (reg) internal address
// Returns true if data was read successfully
extern bool read_i2c (uint16_t slave, uint16_t reg, uint8_t* buf, uint16_t size);
/***********************************************************/

// Current fan state descriptor
// rpm == fan speed in RPM
// value == fan speed in %
// isValid == true if fan speed is valid
typedef struct{
    uint16_t rpm;
    uint8_t value;
    bool isValid;
}MAX6650_FanParameters;

// MAX6650 initialization
// Returns true if initialization finished successfully
// address == I2C bus address
// i2cSpeed == I2C bus speed
// fanVoltage == fan voltage
// minRPM == minimal (1%) fan speed in RPM
// maxRPM == maximal (100%) fan speed in RPM
bool MAX6650_init(MAX6650_Address address,
                  MAX6650_I2CSpeed i2cSpeed,
                  MAX6650_FanVoltage fanVoltage,
                  uint16_t minRPM,
                  uint16_t maxRPM);

// Sets fan speed in %
// Returns true if speed is set successfully
// Fan is off if value == 0
// Fan speed is minRPM if value == 1
// Fan speed is maxRPM if value == MAX6650_MAX_FAN_VALUE
bool MAX6650_setValue(uint8_t value);

// Returns current fan speed in % and RPM
MAX6650_FanParameters MAX6650_getFanParameters();

#endif /* INC_MAX6650_H_ */
