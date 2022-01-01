#pragma once

/* Adafruit_BusIO library by bigdaddyaddie
 */

// This will load the definition for common Particle variable types
#include "Particle.h"

// This is your main class that users will import into their application
class Adafruit_BusIO
{
public:
  /**
   * Constructor
   */
  Adafruit_BusIO();

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
