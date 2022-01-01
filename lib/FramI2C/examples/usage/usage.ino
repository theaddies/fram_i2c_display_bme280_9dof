// Example usage for FramI2C library by ada.

#include "FramI2C.h"

// Initialize objects from the lib
FramI2C framI2C;

void setup() {
    // Call functions on initialized library objects that require hardware
    framI2C.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    framI2C.process();
}
