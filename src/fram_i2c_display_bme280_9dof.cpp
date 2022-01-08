/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "z:/Personal/Electronics/particle/fram_i2c_display_bme280_9dof/src/fram_i2c_display_bme280_9dof.ino"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <imumaths.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_BME280.h>
#include "Adafruit_EEPROM_I2C.h"
#include "Adafruit_FRAM_I2C.h"
#include "math.h"

/* Example code for the Adafruit I2C EEPROM/FRAM breakout */

/* Connect SCL    to SCL
   Connect SDA    to SDA
   Connect VDD    to 3 - 5V DC
   Connect GROUND to common ground */
   


//#include <Adafruit_bno055/utility/imumaths.h>

/* Returns the IMU data as both a euler angles and quaternions as the WebSerial
   3D Model viewer at https://adafruit-3dmodel-viewer.glitch.me/ expects.
 
   This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground

   History
   =======
   2020/JUN/01  - First release (Melissa LeBlanc-Williams)
*/
void setup(void);
void loop(void);
void displaySensorStatus(void);
void displayCalStatus(void);
void printValues();
void displayValues();
void displaySensorDetails(void);
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);
void getHeading(int direction);
void isr_rotation ();
#line 50 "z:/Personal/Electronics/particle/fram_i2c_display_bme280_9dof/src/fram_i2c_display_bme280_9dof.ino"
Adafruit_EEPROM_I2C i2ceeprom;
//Adafruit_FRAM_I2C i2ceeprom;

#define EEPROM_ADDR 0x50  // the default address!

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

// OLED FeatherWing buttons map to different pins depending on board:
#if defined(ESP8266)
  #define BUTTON_A  0
  #define BUTTON_B 16
  #define BUTTON_C  2
#elif defined(ESP32)
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
#elif defined(ARDUINO_STM32_FEATHER)
  #define BUTTON_A PA15
  #define BUTTON_B PC7
  #define BUTTON_C PC5
#elif defined(TEENSYDUINO)
  #define BUTTON_A  4
  #define BUTTON_B  3
  #define BUTTON_C  8
#elif defined(ARDUINO_NRF52832_FEATHER)
  #define BUTTON_A 31
  #define BUTTON_B 30
  #define BUTTON_C 27
#else // 32u4, M0, M4, nrf52840 and 328p
  #define BUTTON_A  4
  #define BUTTON_B  3
  #define BUTTON_C  2
#endif

float compass_heading;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (1000)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/**************************************************************************/
/*
    Wind speed and direction variables
*/
/**************************************************************************/
int VaneValue;// raw analog value from wind vane
int Direction;// translated 0 - 360 direction
int CalDirection;// converted value with offset applied
int LastValue;
uint16_t wind_speed_time_interval= 5000; //value in ms
uint32_t wind_speed_time = 0;
uint8_t vane_pin = A0;
uint8_t vane_switch = D5;
uint8_t wind_pin = D6;
volatile unsigned long Rotations = 0; // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine

float WindSpeed; // speed miles per hour

#define Offset 0;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) {
//wind speed and direction setup
LastValue = 1;

pinMode(vane_pin, INPUT);
pinMode(vane_switch, OUTPUT);
Serial.println("Vane Value\tDirection\tHeading");

pinMode(wind_pin, INPUT);
attachInterrupt(wind_pin, isr_rotation, FALLING);

Serial.println("Davis Wind Speed Test");
Serial.println("Rotations\tMPH");



//variables for calibration read from memory
int test = 55;
    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

  Serial.begin(115200);

    while(!Serial);    // time to get serial running
    Serial.println(F("BME280 test"));
  Serial.println("128x64 OLED FeatherWing test");
  display.begin(0x3C, true); // Address 0x3C default

  Serial.println("OLED begun");

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  display.setRotation(1);
  Serial.println("Button test");

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // text display tests
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
  display.print("Connecting to SSID\n'adafruit':");
  display.print("connected!");
  display.println("IP: 10.0.1.23");
  display.println("Sending val #0");
  display.display(); // actually display all of the above

    unsigned status;
    
    // default settings
    status = bme.begin();  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
  Serial.begin(9600);
  Serial.println("WebSerial 3D Firmware"); Serial.println("");

  //add a test for fram

if (i2ceeprom.begin(0x50)) {  // you can stick the new i2c addr in here, e.g. begin(0x51);
    Serial.println("Found I2C EEPROM");
  } else {
    Serial.println("I2C EEPROM not identified ... check your connections?\r\n");
    while (1) delay(10);
  }
  
  // // Write the first byte to 0xAF
  // uint8_t test = 0xAF;
  // i2ceeprom.write(0x0, test);

  // Try to determine the size by writing a value and seeing if it changes the first byte
  Serial.println("Testing size!");

  uint32_t max_addr;
  for (max_addr = 1; max_addr < 0xFFFF; max_addr++) {
    if (i2ceeprom.read(max_addr) != test)
      continue; // def didnt wrap around yet

    // maybe wraped? try writing the inverse
    if (! i2ceeprom.write(max_addr, (byte)~test)) {
        Serial.print("Failed to write address 0x");
        Serial.println(max_addr, HEX);
    }

    // read address 0x0 again
    uint8_t val0 = i2ceeprom.read(0);

    // re-write the old value
    if (! i2ceeprom.write(max_addr, test)) {
        Serial.print("Failed to re-write address 0x");
        Serial.println(max_addr, HEX);
    }    

    // check if addr 0 was changed
    if (val0 == (byte)~test) {
      Serial.println("Found max address");
      break;
    }
  }
  Serial.print("This EEPROM can store ");
  Serial.print(max_addr);
  Serial.println(" bytes");



  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   
  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
   
  /* Display some basic information on this sensor */
  displaySensorDetails();

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }
    else
    {
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        displaySensorOffsets(calibrationData);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        Serial.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;
    }

    delay(1000);

    /* Display some basic information on this sensor */
    displaySensorDetails();

    /* Optional: Display current status */
    displaySensorStatus();

   /* Crystal must be configured AFTER loading calibration data into BNO055. */
    bno.setExtCrystalUse(true);

    sensors_event_t event;
    bno.getEvent(&event);
    /* always recal the mag as It goes out of calibration very often */
    if (foundCalib){
        Serial.println("Move sensor slightly to calibrate magnetometers");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }
    else
    {
        Serial.println("Please Calibrate Sensor: ");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);

            Serial.print("X: ");
            Serial.print(event.orientation.x, 4);
            Serial.print("\tY: ");
            Serial.print(event.orientation.y, 4);
            Serial.print("\tZ: ");
            Serial.print(event.orientation.z, 4);

            /* Optional: Display calibration status */
            displayCalStatus();

            /* New line for the next sample */
            Serial.println("");

            /* Wait the specified delay before requesting new data */
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }

    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    Serial.println("\n\nStoring calibration data to EEPROM...");

    eeAddress = 0;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    Serial.println("Data stored to EEPROM.");

    Serial.println("\n--------------------------------\n");
    delay(500);
    
  // // validate the memory////////////////////////////////////////////////////////////////////
  // Serial.println("Validating every address in memory");
  // uint8_t val;
  // for (uint16_t addr = 0; addr < max_addr; addr++) {
  //   if (addr % 32 == 0) {
  //     Serial.print("Testing address ");
  //     Serial.print(addr);
  //     Serial.print(" thru ");
  //     Serial.println(addr+31);
  //   }
  //   val = i2ceeprom.read(addr);
    
  //   i2ceeprom.write(addr, 0x55);
  //   if (i2ceeprom.read(addr) != 0x55) {
  //     Serial.print("Failed to write 0x55 to address 0x");
  //     Serial.println(addr, HEX);
  //   }
  //   i2ceeprom.write(addr, 0xAA);
  //   if (i2ceeprom.read(addr) != 0xAA) {
  //     Serial.print("Failed to write 0xAA to address 0x");
  //     Serial.println(addr, HEX);
  //   }
    
  //   i2ceeprom.write(addr, val);
  //   if (i2ceeprom.read(addr) != val) {
  //     Serial.print("Failed to write original value to address 0x");
  //     Serial.println(addr, HEX);
  //   }
  // }
  // Serial.println("Validated!");
  
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) {
//wind speed and direction
digitalWrite(vane_switch, HIGH);
VaneValue = analogRead(vane_pin);
//digitalWrite(vane_switch, LOW);
//Serial.println("analog read value = ");
//Serial.print(VaneValue);
Direction = map(VaneValue, 0, 4095, 0, 360);
CalDirection = Direction + Offset;

if(CalDirection > 360)
CalDirection = CalDirection - 360;

if(CalDirection < 0)
CalDirection = CalDirection + 360;


//delay(100);
if ((millis() - wind_speed_time) > wind_speed_time_interval) {
// Only update the display if change greater than 2 degrees.
  if(abs(CalDirection - LastValue) > 5)
  {
  Serial.print(VaneValue); Serial.print("\t\t");
  Serial.print(CalDirection); Serial.print("\t\t");
  getHeading(CalDirection);
  LastValue = CalDirection;
  }
WindSpeed = Rotations * .45;

Serial.print(Rotations); Serial.print("\t\t");
Serial.print(WindSpeed); Serial.print("\t\t");  Serial.println(" mph");
wind_speed_time = millis();
Rotations = 0;  // Set Rotations count to 0 ready for calculations
// convert to mp/h using the formula V=P(2.25/T)
// V = P(2.25/3) = P * 0.75
}

  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

printValues();
  displayValues();
  delay(delayTime);
  if(!digitalRead(BUTTON_A)) display.print("A");
  if(!digitalRead(BUTTON_B)) display.print("B");
  if(!digitalRead(BUTTON_C)) display.print("C");
  delay(10);
  yield();
  display.display();
  /* The WebSerial 3D Model Viewer expects data as heading, pitch, roll */
  Serial.print(F("Orientation: "));
  Serial.print(360 - (float)event.orientation.x);
  Serial.print(F(", "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(", "));
  Serial.print((float)event.orientation.z);
  Serial.println(F(""));

  /* The WebSerial 3D Model Viewer also expects data as roll, pitch, heading */
  imu::Quaternion quat = bno.getQuat();
  
  Serial.print(F("Quaternion: "));
  Serial.print((float)quat.w());
  Serial.print(F(", "));
  Serial.print((float)quat.x());
  Serial.print(F(", "));
  Serial.print((float)quat.y());
  Serial.print(F(", "));
  Serial.print((float)quat.z());
  Serial.println(F(""));

  /* Also send calibration data for each sensor. */
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print(F("Calibration: "));
  Serial.print(sys, DEC);
  Serial.print(F(", "));
  Serial.print(gyro, DEC);
  Serial.print(F(", "));
  Serial.print(accel, DEC);
  Serial.print(F(", "));
  Serial.print(mag, DEC);
  Serial.println(F(""));

  Serial.println("\n\n");
//  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");

  Serial.println("\n\n");

  compass_heading = event.orientation.x +285;

  if(compass_heading > 360) {
    compass_heading = compass_heading - 360;
  }
  
  Serial.print("compass heading:  ");
  Serial.print(compass_heading, 4);



      adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

      delay(BNO055_SAMPLERATE_DELAY_MS);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/

void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

void printValues() {
    Serial.print("\n\nTemperature = ");
    Serial.print(bme.readTemperature()*1.8F + 32.);
    Serial.println(" F");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 101325.0F * 760.0F);
    Serial.println(" mmHg");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");

    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}

void displayValues() {
    display.clearDisplay();
  display.display();
  display.setCursor(0,0);
    display.print("Temp. = ");
    display.print(bme.readTemperature()*1.8F + 32.);
    display.println(" °F");

    display.print("Press. = ");

    display.print(bme.readPressure() / 101325.0F * 760.0F);
    display.println(" mmHg");

    display.print("Altitude = ");
    display.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    display.println(" m");

    display.print("Humidity = ");

    display.print(bme.readHumidity());
    display.println(" %");

    Serial.println();
    display.println("Marrie is great!");
    Serial.println();
    display.print("Compass heading");
    display.print(compass_heading);
  display.display(); // actually display all of the above
  }
  
  /**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.println("\nCalibration offsets \n");
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}
// Converts compass direction to heading
void getHeading(int direction) {
if(direction < 22)
Serial.println("N");
else if (direction < 67)
Serial.println("NE");
else if (direction < 112)
Serial.println("E");
else if (direction < 157)
Serial.println("SE");
else if (direction < 212)
Serial.println("S");
else if (direction < 247)
Serial.println("SW");
else if (direction < 292)
Serial.println("W");
else if (direction < 337)
Serial.println("NW");
else
Serial.println("N");
} 

// This is the function that the interrupt calls to increment the rotation count
void isr_rotation () {

if ((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact.
Rotations++;
ContactBounceTime = millis();
}

}
