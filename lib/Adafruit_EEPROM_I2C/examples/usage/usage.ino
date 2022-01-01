// Example usage for Adafruit_EEPROM_I2C library by ada.

#include "Adafruit_EEPROM_I2C.h"

// Initialize objects from the lib
Adafruit_EEPROM_I2C adafruit_EEPROM_I2C;

void setup() {
    // Call functions on initialized library objects that require hardware
    adafruit_EEPROM_I2C.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    adafruit_EEPROM_I2C.process();
}
