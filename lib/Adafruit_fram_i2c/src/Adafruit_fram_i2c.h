#pragma once

/* Adafruit_FRAM_I2C library by asdf
 */

// This will load the definition for common Particle variable types
#include "Particle.h"

// This is your main class that users will import into their application
class Adafruit_FRAM_I2C
{
public:
  /**
   * Constructor
   */
  Adafruit_FRAM_I2C();

  /**
   * Example method
   */
  void begin();

  /**
   * Example method
   */
  void process();

private:
  /**
   * Example private method
   */
  void doit();
};
