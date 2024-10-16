# ISL28022 ESP IDF Driver

This is a driver written for the ISL28022 DPM (Digital Power Monitor) device, for use with an ESP32 using the ESP-IDF framework version 5.3.1.

Like with most I2C interfacing ICs, this device is accessed by reading from and writing to various registers in the device.

This driver is intended to be included in your ESP-IDF project as a 'component' and used accordingly.

Although many Drivers built around the Arduino framework exist for this chip, I could not find one built around the ESP-IDF framework.

Enjoy :D

## Usage

1. Create an instance of an I2C master bus.
2. Create an instance of an I2C master device, and register it with the previously created bus.
3. Call the 'isl28022_create' fxn, and pass it the device handle associated with the ISL28022 previously instanitated, along with the value of the shunt resistor used in your particular circuit implementation of the ISL28022.
4. Call the 'isl28022_init' fxn, and pass it the isl28022 driver device struct returned by the previous 'isl28022_create' fxn call
5. Declare a isl28022_config_t struct and define the appropriate config register values for your implementation.
6. Call the 'isl28022_set_config' fxn and pass it the previously defined config struct.
7. Call the 'isl28022_set_calibration_reg' fxn to set the calibration register values and calculate current and power LSB values needed for accurate current and power calc readings.
8. Now you can read values and access registers however needed for your application.

See an example usage in the example directory [here](examples/config_isl28022_and_read.c)

## Notes

This driver was written to be used by the (newer) i2c_master.h esp idf driver, which as of framework version 5.3 replaced the original i2c.h driver. The i2c.h driver is now depricated.

ESP-IDF version 5.3.1 is the latest stable release of the ESP-IDF at the time of creation of this driver.

## TODOs

- [ ] FIX Power reading fxn
- [ ] Implement missing funtionality
  - [ ] Shunt voltage register get
  - [ ] Threshold register get and set
  - [ ] AUX control register get and set
- [ ] Add a 'self test' fxn
- [ ] Thoroughly test each function
- [ ] Add SDKCONFIG and allow easy config of params using menuconfig
