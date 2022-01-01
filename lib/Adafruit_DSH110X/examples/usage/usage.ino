// Example usage for Adafrtuit_DSH110X library by ada.

#include "Adafrtuit_DSH110X.h"

// Initialize objects from the lib
Adafrtuit_DSH110X adafrtuit_DSH110X;

void setup() {
    // Call functions on initialized library objects that require hardware
    adafrtuit_DSH110X.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    adafrtuit_DSH110X.process();
}
