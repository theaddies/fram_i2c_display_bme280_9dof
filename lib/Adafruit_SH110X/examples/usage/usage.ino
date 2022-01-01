// Example usage for Adafruit_SH110X library by ada.

#include "Adafruit_SH110X.h"

// Initialize objects from the lib
Adafruit_SH110X adafruit_SH110X;

void setup() {
    // Call functions on initialized library objects that require hardware
    adafruit_SH110X.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    adafruit_SH110X.process();
}
