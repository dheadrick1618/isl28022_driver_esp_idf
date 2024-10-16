/**
 * @file isl28022_driver.c
 * @brief Implementation of the ISL28022 Digital Power Monitor driver
 *
 * @copyright Copyright (c) 2024 Devin Headrick
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 */

#include <stdio.h>
#include <esp_log.h>
#include <math.h>
#include <freertos/FreeRTOS.h>

#include "isl28022_driver.h"

static const char *TAG = "ISL28022 driver";

/// ------------------------ Static Helper fxns -------------------------- ///

static float calculate_current_full_scale(float v_shunt_fs, float r_shunt)
{
    return v_shunt_fs / r_shunt;
}

static float calculate_current_lsb(float current_full_scale, uint32_t adc_resolution)
{
    return current_full_scale / adc_resolution;
}

static uint16_t calculate_calibration_value(float current_lsb, float r_shunt)
{
    // Using equation from datasheet: Cal = trunc(0.04096 / (Current_LSB * R_SHUNT))
    float cal_float = 0.04096 / (current_lsb * r_shunt);
    return (uint16_t)cal_float;
}

/// @brief Write to a register of the ISL28022 using the I2C bus
/// @param address
/// @param value
/// @note Write op requires: START condition, id byte, address byte, two data bytes, and STOP condition
/// @note First data byte contains MSB of data. Last data byte contains MLSB of data.
/// @note After each byte, the ISL28022 responds with an ACK, then the I2C interface enters a standby state.
/// @return
static esp_err_t isl28022_write_reg(isl28022_driver_t *isl28022_dev, uint8_t reg_addr, uint16_t reg_value)
{
    if (isl28022_dev == NULL || reg_addr > AUX_CONTROL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t write_bytes[3];
    write_bytes[0] = reg_addr;
    write_bytes[1] = (uint8_t)(reg_value >> 8);
    write_bytes[2] = (uint8_t)(reg_value & 0xFFU);

    esp_err_t err = i2c_master_transmit(isl28022_dev->dev_handle, write_bytes, sizeof(write_bytes), pdMS_TO_TICKS(I2C_WRITE_TIMEOUT_MS));

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write register 0x%02x: %s", reg_addr, esp_err_to_name(err));
    }
    return err;
}

/// @brief Read from a register of the ISL28022 using the I2C bus
/// @param isl28022_dev
/// @param reg_addr
/// @param dout
/// @param size
/// @note
/// @return
static esp_err_t isl28022_read_reg(isl28022_driver_t *isl28022_dev, uint8_t reg_addr, uint16_t *reg_value)
{
    if (isl28022_dev == NULL || reg_addr > AUX_CONTROL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t bytes_read[2];

    esp_err_t err = i2c_master_transmit(isl28022_dev->dev_handle, &reg_addr, 1, pdMS_TO_TICKS(I2C_WRITE_TIMEOUT_MS));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write register address 0x%02X: %s", reg_addr, esp_err_to_name(err));
        return err;
    }

    err = i2c_master_receive(isl28022_dev->dev_handle, bytes_read, sizeof(bytes_read), pdMS_TO_TICKS(I2C_READ_TIMEOUT_MS));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write register address 0x%02X: %s", reg_addr, esp_err_to_name(err));
        return err;
    }

    *reg_value = ((uint16_t)bytes_read[0] << 8) | bytes_read[1];

    return err;
}
/// ---------------------------------------------------------------------- ///

isl28022_driver_t *isl28022_create(i2c_master_dev_handle_t dev_handle, float shunt_resistor_ohms)
{
    if (dev_handle == NULL || shunt_resistor_ohms <= 0.0)
    {
        ESP_LOGE(TAG, "Invalid parameters");
        return NULL;
    }

    isl28022_driver_t *isl28022_dev = calloc(1, sizeof(isl28022_driver_t));
    if (isl28022_dev == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for driver");
        return NULL;
    }

    isl28022_dev->dev_handle = dev_handle;
    isl28022_dev->shunt_resistor_ohms = shunt_resistor_ohms;

    return isl28022_dev;
}

void isl28022_destroy(isl28022_driver_t *isl28022_dev)
{
    free(isl28022_dev);
}

esp_err_t isl28022_init(isl28022_driver_t *isl28022_dev)
{
    // TODO - probe for device - verify device is on the i2c bus using its slave address

    esp_err_t err = isl28022_reset(isl28022_dev);
    return err;
}

esp_err_t isl28022_reset(isl28022_driver_t *isl28022_dev)
{
    if (isl28022_dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t reset_reg_value = 0x8000U;
    esp_err_t err = isl28022_write_reg(isl28022_dev, CONFIG, reset_reg_value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to reset device: %s", esp_err_to_name(err));
        return err;
    }

    // Add a delay to allow the device to reset, and system to callibrate
    vTaskDelay(pdMS_TO_TICKS(ISL28022_RESET_DELAY_MS));

    return ESP_OK;
}

esp_err_t isl28022_get_config(isl28022_driver_t *dev, uint16_t *register_bits)
{
    if (dev == NULL || register_bits == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t reg_value;
    esp_err_t err = isl28022_read_reg(dev, CONFIG, &reg_value);
    if (err == ESP_OK)
    {
        *register_bits = reg_value;
        ESP_LOGI(TAG, "Config register read value: 0x%04x", *register_bits);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read config register");
    }
    return err;
}

esp_err_t isl28022_set_config(isl28022_driver_t *dev, const isl28022_config_t *config)
{
    if (dev == NULL || config == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t config_value = (uint16_t)((config->brng << BRNG0) |
                                       (config->pg << PGA0) |
                                       (config->badc << BADC0) |
                                       (config->sadc << SADC0) |
                                       config->mode);

    esp_err_t err = isl28022_write_reg(dev, CONFIG, config_value);
    return err;
}

esp_err_t isl28022_get_calibration(isl28022_driver_t *isl28022_dev, uint16_t *register_bits)
{
    if (isl28022_dev == NULL || register_bits == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t reg_value;
    esp_err_t err = isl28022_read_reg(isl28022_dev, CALIB, &reg_value);
    if (err == ESP_OK)
    {
        *register_bits = reg_value;
        ESP_LOGI(TAG, "Config register read value: 0x%04x", *register_bits);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read config register");
    }
    return err;
}

esp_err_t isl28022_set_calibration(isl28022_driver_t *dev)
{
    if (dev == NULL || dev->shunt_resistor_ohms <= 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    // Step 1: Read the configuration register to get PGA and SADC settings
    uint16_t reg_value;
    esp_err_t err = isl28022_read_reg(dev, CONFIG, &reg_value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read configuration register: %s", esp_err_to_name(err));
        return err;
    }

    // Extract PGA and SADC settings
    isl28022_pg_t pga = (reg_value >> PGA0) & 0x03;
    isl28022_sadc_t sadc = (reg_value >> SADC0) & 0x0F;

    // Determine V_SHUNT_FS based on PGA setting
    float v_shunt_fs;
    switch (pga)
    {
    case PGA_1_40MV:
        v_shunt_fs = 0.04;
        break;
    case PGA_2_80MV:
        v_shunt_fs = 0.08;
        break;
    case PGA_4_160MV:
        v_shunt_fs = 0.16;
        break;
    case PGA_8_320MV:
        v_shunt_fs = 0.32;
        break;
    default:
        return ESP_ERR_INVALID_STATE;
    }

    // Determine ADC resolution based on SADC setting
    uint32_t adc_resolution;
    switch (sadc)
    {
    case SADC_12BIT:
        adc_resolution = 1 << 12; // 12 bit ADC accuracy
        break;
    case SADC_13BIT:
        adc_resolution = 1 << 13;
        break;
    case SADC_14BIT:
        adc_resolution = 1 << 14;
        break;
    // case SADC_15BIT:
    //     adc_resolution = 1 << 15;
    //     break;
    // case SADC_15BIT_1:
    //     adc_resolution = 1 << 15;
    //     break;
    default:
        adc_resolution = 1 << 15;
        break;
    }

    // Step 2: Calculate the Current Full Scale
    float current_full_scale = calculate_current_full_scale(v_shunt_fs, dev->shunt_resistor_ohms);

    // Step 3: Calculate the Current_LSB
    float current_lsb = calculate_current_lsb(current_full_scale, adc_resolution);

    // Step 4: Calculate the Calibration value
    uint16_t cal_reg_value = calculate_calibration_value(current_lsb, dev->shunt_resistor_ohms);

    // Step 5: Write the Calibration value to the Calibration Register
    err = isl28022_write_reg(dev, CALIB, cal_reg_value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write calibration register: %s", esp_err_to_name(err));
        return err;
    }

    // Step 6: Calculate and store the Power_LSB
    float power_lsb = 20 * current_lsb;

    // Store calculated values in the device structure for future use
    dev->current_lsb = current_lsb;
    dev->power_lsb = power_lsb;

    ESP_LOGI(TAG, "Calibration set: Cal = %u, Current_FS = %.6f A, Current_LSB = %.9f A, Power_LSB = %.9f W",
             cal_reg_value, current_full_scale, current_lsb, power_lsb);

    return err;
}

esp_err_t isl28022_read_bus_voltage(isl28022_driver_t *isl28022_dev, float *voltage_reading)
{
    if (isl28022_dev == NULL || voltage_reading == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Get the BRNG bits from config register
    uint16_t reg_value;
    esp_err_t err = isl28022_read_reg(isl28022_dev, CONFIG, &reg_value);
    if (err != ESP_OK)
    {
        return err;
    }
    isl28022_brng_t brng = (reg_value >> BRNG0) & 0x03;

    // Get the bus voltage register bits
    err = isl28022_read_reg(isl28022_dev, BUS_V, &reg_value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read bus voltage register");
        return err;
    }

    // Calculate the bus voltage using register bits and bit weights associated with the BRNG setting
    uint32_t vbus = 0;
    uint16_t bit_mask;
    uint16_t bit_weight;

    switch (brng)
    {
    case BRNG_16V:
        // 12 bits wide, starting from bit 14
        for (int i = 0; i < 12; i++)
        {
            bit_mask = 1 << (14 - i);
            bit_weight = 1 << (11 - i); // Weight starts at 2048 (2^11) -> 1 (2^0)
            if (reg_value & bit_mask)
            {
                vbus += bit_weight;
            }
        }
        break;
    case BRNG_32V:
        // 13 bits wide, starting from bit 15
        for (int i = 0; i < 13; i++)
        {
            bit_mask = 1 << (15 - i);
            bit_weight = 1 << (12 - i); // Weight starts at 4096 (2^12) -> 1 (2^0)
            if (reg_value & bit_mask)
            {
                vbus += bit_weight;
            }
        }
        break;
    case BRNG_60V:
    default:
        // 14 bits wide, starting from bit 15
        for (int i = 0; i < 14; i++)
        {
            bit_mask = 1 << (15 - i);
            bit_weight = 1 << (13 - i); // Weight starts at 8192 (2^13) -> 1 (2^0)
            if (reg_value & bit_mask)
            {
                vbus += bit_weight;
            }
        }
        break;
    }
    *voltage_reading = (float)vbus * ISL28022_BUS_VOLTAGE_LSB_MILLIVOLT;
    return err;
}

esp_err_t isl28022_read_current(isl28022_driver_t *isl28022_dev, float *current_reading)
{
    if (isl28022_dev == NULL || current_reading == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t reg_value;
    esp_err_t err = isl28022_read_reg(isl28022_dev, CURRENT, &reg_value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read current register");
        return err;
    }

    // Calculate the current using the register reading
    float current = 0;

    // The current register is always 16 bits wide, with the MSB being the sign bit
    for (int i = 0; i < 16; i++)
    {
        int16_t bit_weight = 1 << (15 - i);
        if (reg_value & bit_weight)
        {
            if (i == 0)
            {
                // Handle the sign bit
                current -= 32768;
            }
            else
            {
                current += bit_weight;
            }
        }
    }
    current *= isl28022_dev->current_lsb;
    *current_reading = current;
    return err;
}

esp_err_t isl28022_read_power(isl28022_driver_t *isl28022_dev, float *power_reading)
{
    if (isl28022_dev == NULL || power_reading == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t reg_value;
    esp_err_t err = isl28022_read_reg(isl28022_dev, POWER, &reg_value);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read current register");
        return err;
    }

    float power = 0;
    for (int i = 0; i < 16; i++)
    {
        int16_t bit_weight = 1 << (15 - i);
        if (reg_value & bit_weight)
        {
            power += bit_weight;
        }
    }
    power *= isl28022_dev->power_lsb;
    power *= 5000; // Data sheet specifies to multiply resulting value by 5000, but does not give context for why
    // TODO - if Vbus range (BRNG) is set to 60V, the result is multiplied by 2 (this involes reading the config register to get brng value)
    *power_reading = power;
    return err;
}