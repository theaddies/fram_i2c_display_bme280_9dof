// Example usage for imumaths library by ada.

#include "imumaths.h"

// Initialize objects from the lib
Imumaths imumaths;

void setup() {
    // Call functions on initialized library objects that require hardware
    imumaths.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    imumaths.process();
}
