// Example usage for Adafruit_BusIO library by ada.

#include "Adafruit_BusIO.h"

// Initialize objects from the lib
Adafruit_BusIO adafruit_BusIO;

void setup() {
    // Call functions on initialized library objects that require hardware
    adafruit_BusIO.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    adafruit_BusIO.process();
}
