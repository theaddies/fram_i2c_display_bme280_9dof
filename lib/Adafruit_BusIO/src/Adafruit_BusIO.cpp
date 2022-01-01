/* Adafruit_BusIO library by bigdaddyaddie
 */

#include "Adafruit_BusIO.h"

/**
 * Constructor.
 */
Adafruit_BusIO::Adafruit_BusIO()
{
  // be sure not to call anything that requires hardware be initialized here, put those in begin()
}

/**
 * Example method.
 */
void Adafruit_BusIO::begin()
{
    // initialize hardware
    Serial.println("called begin");
}

/**
 * Example method.
 */
void Adafruit_BusIO::process()
{
    // do something useful
    Serial.println("called process");
    doit();
}

/**
* Example private method
*/
void Adafruit_BusIO::doit()
{
    Serial.println("called doit");
}
