#include <esp_err.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>

#include "isl28022_driver.h"

#define I2C_MASTER_SCL_IO 13               // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO 5                // GPIO number for I2C master data
#define I2C_MASTER_NUM 0                   // I2C master i2c port number
#define I2C_MASTER_FREQ_HZ 100000          // I2C master clock frequency
#define ISL28022_ADDRESS 0x40              // 100 0000  (64)     is 7 dig slave address as per data sheet when both address pins are grounded
#define ISL28022_SHUNT_RESISTOR_OHMS 0.160 // 160mOhms shunt resistor value used by this circuit

static const char *TAG = "Config ISL28022";

i2c_master_bus_handle_t config_i2c_bus()
{
    i2c_master_bus_config_t i2c_master_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_bus_config, &i2c_bus_handle));
    ESP_LOGI(TAG, "I2C master bus instantaited");
    return i2c_bus_handle;
}

i2c_master_dev_handle_t config_i2c_device(uint16_t dev_address, i2c_master_bus_handle_t bus_handle)
{
    i2c_device_config_t bme280_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = dev_address,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &bme280_config, &dev_handle));
    ESP_LOGI(TAG, "I2C bme280 device instantiated and added to bus instance");
    return dev_handle;
}

void app_main(void)
{
    i2c_master_bus_handle_t bus_handle = config_i2c_bus();
    vTaskDelay(pdMS_TO_TICKS(1000));
    i2c_master_dev_handle_t dev_handle = config_i2c_device((uint16_t)ISL28022_ADDRESS, bus_handle);

    //--------------------------------------------------------------------
    // ISL28022 SETUP CODE BEGINS HERE

    isl28022_driver_t *isl28022_dev = isl28022_create(dev_handle, ISL28022_SHUNT_RESISTOR_OHMS);
    if (!isl28022_dev)
    {
        ESP_LOGE(TAG, "Could not create ISL28022 driver");
        return;
    }
    else
    {
        ESP_LOGI(TAG, "ISL28022 Driver created");
    }

    esp_err_t err = isl28022_init(isl28022_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize ISL28022: %s", esp_err_to_name(err));
        isl28022_destroy(isl28022_dev);
        return;
    }

    // THESE SETTINGS ARE SPECIFIC TO THE HARDWARE / CIRCUIT CONFIGURATION - if this is not done all operations will be assuming default device register values
    isl28022_config_t config = {
        .brng = BRNG_32V,
        .pg = PGA_4_160MV,
        .badc = BADC_15BIT,
        .sadc = SADC_15BIT,
        .mode = MODE_SHUNT_BUS_CONT};

    isl28022_set_config(isl28022_dev, &config);

    isl28022_set_calibration(isl28022_dev);

    float voltage_reading;
    float current_reading;
    float power_reading;
    for (;;)
    {
        err = isl28022_read_bus_voltage(isl28022_dev, &voltage_reading);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Voltage reading is : %.3f V", voltage_reading);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read bus voltage");
        }
        vTaskDelay(pdMS_TO_TICKS(500));

        err = isl28022_read_current(isl28022_dev, &current_reading);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Current reading is : %.3f A", current_reading);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read current");
        }
        vTaskDelay(pdMS_TO_TICKS(500));

        err = isl28022_read_power(isl28022_dev, &power_reading);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Power reading is : %.3f W", power_reading);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read power");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}