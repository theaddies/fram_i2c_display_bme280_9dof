// Example usage for Adafruit_GFX library by ada.

#include "Adafruit_GFX.h"

// Initialize objects from the lib
Adafruit_GFX adafruit_GFX;

void setup() {
    // Call functions on initialized library objects that require hardware
    adafruit_GFX.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    adafruit_GFX.process();
}
