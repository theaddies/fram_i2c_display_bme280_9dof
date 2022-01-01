// Example usage for EEPROM_I2C library by ada.

#include "EEPROM_I2C.h"

// Initialize objects from the lib
EEPROM_I2C eEPROM_I2C;

void setup() {
    // Call functions on initialized library objects that require hardware
    eEPROM_I2C.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    eEPROM_I2C.process();
}
