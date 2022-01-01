/* Adafrtuit_DSH110X library by ada
 */

#include "Adafrtuit_DSH110X.h"

/**
 * Constructor.
 */
Adafrtuit_DSH110X::Adafrtuit_DSH110X()
{
  // be sure not to call anything that requires hardware be initialized here, put those in begin()
}

/**
 * Example method.
 */
void Adafrtuit_DSH110X::begin()
{
    // initialize hardware
    Serial.println("called begin");
}

/**
 * Example method.
 */
void Adafrtuit_DSH110X::process()
{
    // do something useful
    Serial.println("called process");
    doit();
}

/**
* Example private method
*/
void Adafrtuit_DSH110X::doit()
{
    Serial.println("called doit");
}
