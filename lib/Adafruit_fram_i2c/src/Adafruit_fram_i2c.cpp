/* Adafruit_FRAM_I2C library by asdf
 */

#include "Adafruit_FRAM_I2C.h"

/**
 * Constructor.
 */
Adafruit_FRAM_I2C::Adafruit_FRAM_I2C()
{
  // be sure not to call anything that requires hardware be initialized here, put those in begin()
}

/**
 * Example method.
 */
void Adafruit_FRAM_I2C::begin()
{
    // initialize hardware
    Serial.println("called begin");
}

/**
 * Example method.
 */
void Adafruit_FRAM_I2C::process()
{
    // do something useful
    Serial.println("called process");
    doit();
}

/**
* Example private method
*/
void Adafruit_FRAM_I2C::doit()
{
    Serial.println("called doit");
}
