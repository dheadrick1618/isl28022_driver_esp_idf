/**
 * @file isl28022_driver.h
 * @brief Driver for the ISL28022 Digital Power Monitor
 *
 * @copyright Copyright (c) 2024 Devin Headrick
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 */

#pragma once

#include <esp_err.h>
#include <driver/i2c_master.h>

// TODO - Modify this to be configured via sdkconfig (menuconfig)
#define I2C_WRITE_TIMEOUT_MS 100
#define I2C_READ_TIMEOUT_MS 100

#define ISL28022_RESET_DELAY_MS 10
#define ISL28022_BUS_VOLTAGE_LSB_MILLIVOLT 0.004 // 4mV

/// Used for bit shifts associated with the config register
enum ISL28022_CONFIG_REG_BITS
{
    RST = 15,
    BRNG1 = 14, // Bus voltage range MSB
    BRNG0 = 13, // Bus voltage range LSB
    PGA1 = 12,  // shunt voltage range (pga gain) MSB
    PGA0 = 11,  // shunt voltage range (pga gain) LSB
    BADC3 = 10, // Vbus ADS resolution MSB
    BADC2 = 9,
    BADC1 = 8,
    BADC0 = 7, // Vbus ADC resolution LSD
    SADC3 = 6, // Shunt ADC resolution / Averaging MSB
    SADC2 = 5,
    SADC1 = 4,
    SADC0 = 3, // Shunt ADC resolution / Averaging LSB
    MODE2 = 2, // Operating mode MSB
    MODE1 = 1,
    MODE0 = 0 // Operating mode LSB
};

// Bus Voltage Range (BRNG)
typedef enum
{
    BRNG_16V = 0b00, // 16V FSR
    BRNG_32V = 0b01, // 32V FSR
    BRNG_60V = 0b10  // 60V FSR (also 0b11)
} isl28022_brng_t;

// PGA Gain and Range (PG)
typedef enum
{
    PGA_1_40MV = 0b00,  // x1 gain, ±40mV range
    PGA_2_80MV = 0b01,  // x1/2 gain, ±80mV range
    PGA_4_160MV = 0b10, // x1/4 gain, ±160mV range
    PGA_8_320MV = 0b11  // x1/8 gain, ±320mV range
} isl28022_pg_t;

// Bus ADC Resolution/Averaging (BADC)
typedef enum
{
    BADC_12BIT = 0b0000,       // 12-bit, 72µs
    BADC_13BIT = 0b0001,       // 13-bit, 132µs
    BADC_14BIT = 0b0010,       // 14-bit, 258µs
    BADC_15BIT = 0b0011,       // 15-bit, 508µs
    BADC_15BIT_1 = 0b1000,     // 15-bit, 508µs
    BADC_16BIT_2AVG = 0b1001,  // 16-bit, 1.01ms, 2 samples averaged
    BADC_16BIT_4AVG = 0b1010,  // 16-bit, 2.01ms, 4 samples averaged
    BADC_16BIT_8AVG = 0b1011,  // 16-bit, 4.01ms, 8 samples averaged
    BADC_16BIT_16AVG = 0b1100, // 16-bit, 8.01ms, 16 samples averaged
    BADC_16BIT_32AVG = 0b1101, // 16-bit, 16.01ms, 32 samples averaged
    BADC_16BIT_64AVG = 0b1110, // 16-bit, 32.01ms, 64 samples averaged
    BADC_16BIT_128AVG = 0b1111 // 16-bit, 64.01ms, 128 samples averaged
} isl28022_badc_t;

// Shunt ADC Resolution/Averaging (SADC)
typedef enum
{
    SADC_12BIT = 0b0000,       // 12-bit, 72µs
    SADC_13BIT = 0b0001,       // 13-bit, 132µs
    SADC_14BIT = 0b0010,       // 14-bit, 258µs
    SADC_15BIT = 0b0011,       // 15-bit, 508µs
    SADC_15BIT_1 = 0b1000,     // 15-bit, 508µs
    SADC_16BIT_2AVG = 0b1001,  // 16-bit, 1.01ms, 2 samples averaged
    SADC_16BIT_4AVG = 0b1010,  // 16-bit, 2.01ms, 4 samples averaged
    SADC_16BIT_8AVG = 0b1011,  // 16-bit, 4.01ms, 8 samples averaged
    SADC_16BIT_16AVG = 0b1100, // 16-bit, 8.01ms, 16 samples averaged
    SADC_16BIT_32AVG = 0b1101, // 16-bit, 16.01ms, 32 samples averaged
    SADC_16BIT_64AVG = 0b1110, // 16-bit, 32.01ms, 64 samples averaged
    SADC_16BIT_128AVG = 0b1111 // 16-bit, 64.01ms, 128 samples averaged
} isl28022_sadc_t;

// Operating Mode (MODE)
typedef enum
{
    MODE_POWER_DOWN = 0b000,     // Power-down
    MODE_SHUNT_TRIG = 0b001,     // Shunt voltage, triggered
    MODE_BUS_TRIG = 0b010,       // Bus voltage, triggered
    MODE_SHUNT_BUS_TRIG = 0b011, // Shunt and bus, triggered
    MODE_ADC_OFF = 0b100,        // ADC off (disabled)
    MODE_SHUNT_CONT = 0b101,     // Shunt voltage, continuous
    MODE_BUS_CONT = 0b110,       // Bus voltage, continuous
    MODE_SHUNT_BUS_CONT = 0b111  // Shunt and bus, continuous
} isl28022_mode_t;

enum ISL28022_REGISTER_ADDRESSES
{
    CONFIG = 0x00,
    SHUNT_V = 0x01,
    BUS_V = 0x02,
    POWER = 0x03,
    CURRENT = 0x04,
    CALIB = 0x05,
    SHUNT_V_THRESH = 0x06,
    BUS_V_THRESH = 0x07,
    DCS_INT_STATUS = 0x08,
    AUX_CONTROL = 0x09
};

typedef struct isl28022_driver_t
{
    i2c_master_dev_handle_t dev_handle;
    float shunt_resistor_ohms; // used by calibration reg
    float current_lsb;         // used by the current register read calc
    float power_lsb;           // used by the power regsiter read calc
} isl28022_driver_t;

typedef struct isl28022_config_t
{
    isl28022_brng_t brng;
    isl28022_pg_t pg;
    isl28022_badc_t badc;
    isl28022_sadc_t sadc;
    isl28022_mode_t mode;
} isl28022_config_t;

/// @brief Create the ISL28022 device struct. Malloc mem needed for struct and init parameters.
/// @param dev_address 7bit address of the device determined by the configuration of address registers A1 and A0
/// @param dev_handle Already initialized device handle for the isl28022. It is assumed the caller has already registered this device with the master bus.
/// @return
isl28022_driver_t *isl28022_create(i2c_master_dev_handle_t dev_handle, float shunt_resistor_ohms);

/// @brief Free heap that was malloc for the isl28022 device struct
/// @param isl28022_dev
void isl28022_destroy(isl28022_driver_t *isl28022_dev);

/// @brief Probe for the device address on the I2C bus, and reset the device registers using the config register
/// @return
esp_err_t isl28022_init(isl28022_driver_t *isl28022_dev);

/// @brief Set the reset bit (15th) in the config register to '1'
/// @param dev_handle
/// @note Also makes device perform system callibration
/// @return
esp_err_t isl28022_reset(isl28022_driver_t *isl28022_dev);

/// @brief Get the Config register bits the device is currently set to
/// @param dev
/// @param register_bits
/// @note After creating and initializing the device, this should be THE FIRST fxn called before anything else
/// @return
esp_err_t isl28022_get_config(isl28022_driver_t *dev, uint16_t *register_bits);

/// @brief Set the Config register according to the specific configuration of the circuit using the ISL28022 chip
/// @param dev
/// @param brng
/// @param pg
/// @param badc
/// @param sadc
/// @param mode
/// @note See enums in the isl28022_driver.h file to determine what config values to use
/// @return
esp_err_t isl28022_set_config(isl28022_driver_t *dev, const isl28022_config_t *config);

/// @brief Get the Calibration register bits the device is current set to
/// @param isl28022_dev
/// @param register_bits
/// @return
esp_err_t isl28022_get_calibration(isl28022_driver_t *isl28022_dev, uint16_t *register_bits);

/// @brief Set the Calibration register bits resulting from calculations done in this fxn based on Config register values
/// @param dev
/// @note The Config register bits affect the calculations performed by this fxn
/// @note It is assumed that the Config register has been set BEFORE this fxn is called
/// @return
esp_err_t isl28022_set_calibration(isl28022_driver_t *dev);

// TODO - Implement Shunt voltage register get

/// @brief Reads config reg to get BRNG bits to determine what register bit weights to use, then calculated bus voltage according to data sheet equation 1
/// @param isl28022_dev
/// @param voltage_reading Voltage float in Volts
/// @note Performs a read operation on the config register to get BRNG bits
/// @return
esp_err_t isl28022_read_bus_voltage(isl28022_driver_t *isl28022_dev, float *voltage_reading);

/// @brief Calculate the current value using assigned weights to each bit in the register.
/// @param isl28022_dev
/// @param current_reading
/// @note the current LSB value needed in the calc requires the calibration register to have been set
/// @note The 'current LSB' needed in this calculation depends on your circuit configuration
/// @return
esp_err_t isl28022_read_current(isl28022_driver_t *isl28022_dev, float *current_reading);

/// @brief Calcualte the power used by the device (from the aux power source)
/// @param isl28022_dev
/// @param power_reading
/// @note The 'power LSB' values used in the calculations in this fxn require the calibration register to have been set
/// @return
esp_err_t isl28022_read_power(isl28022_driver_t *isl28022_dev, float *power_reading);

// TODO - Implement set and get and threshold registers

// TODO - Implement AUX control register get and set