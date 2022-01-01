// Example usage for Adafruit_fram_i2c library by ada.

#include "Adafruit_fram_i2c.h"

// Initialize objects from the lib
Adafruit_fram_i2c adafruit_fram_i2c;

void setup() {
    // Call functions on initialized library objects that require hardware
    adafruit_fram_i2c.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    adafruit_fram_i2c.process();
}
