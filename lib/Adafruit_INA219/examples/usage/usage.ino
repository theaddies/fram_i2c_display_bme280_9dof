// Example usage for Adafruit_INA219 library by ada.

#include "Adafruit_INA219.h"

// Initialize objects from the lib
Adafruit_INA219 adafruit_INA219;

void setup() {
    // Call functions on initialized library objects that require hardware
    adafruit_INA219.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    adafruit_INA219.process();
}
