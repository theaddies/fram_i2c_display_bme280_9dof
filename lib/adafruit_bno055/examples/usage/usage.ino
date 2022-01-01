// Example usage for adafruit_bno055 library by ada.

#include "adafruit_bno055.h"

// Initialize objects from the lib
Adafruit_bno055 adafruit_bno055;

void setup() {
    // Call functions on initialized library objects that require hardware
    adafruit_bno055.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    adafruit_bno055.process();
}
