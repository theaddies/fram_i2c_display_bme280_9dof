/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "z:/Personal/Electronics/particle/i2c_display_bme280_9dof/src/i2c_display_bme280_9dof.ino"
#include <Wire.h>
// #include <imumaths.h>
#include <SPI.h>
#include <Adafruit_INA219.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_BME280.h>
#include "Adafruit_EEPROM_I2C.h"
#include "Adafruit_FRAM_I2C.h"
#include "math.h"
#include <SparkFunLSM9DS1.h>
#include "OneWire.h"
#include "DS18.h"
void setup(void);
void loop();
void read_bme_values(float& bme_temperature, float& bme_pressure, float& bme_humidity, float& bme_altitude);
void printValues(float& bme_temperature, float& bme_pressure, float& bme_humidity, float& bme_altitude);
void displayValues1(int32_t bme_temperature_average, int32_t bme_pressure_average, int32_t bme_humidity_average, int32_t bme_altitude_average, float busvoltage_average, int32_t current_mA_average, int32_t power_mW_average);
void displayValues2(int8_t water_temperature_average, int32_t vane_wind_direction_average, String& heading  , float& WindSpeed);
void isr_rotation ();
int get_heading(float acc[3], float mag[3], float p[3]);
void get_scaled_IMU(float Axyz[3], float Mxyz[3]);
void vector_cross(float a[3], float b[3], float out[3]);
float vector_dot(float a[3], float b[3]);
void vector_normalize(float a[3]);
uint16_t measure_wind_direction();
String calculateHeading(int direction);
void measure_current_voltage_power(float& shuntvoltage, float& busvoltage, float& current_mA, float& loadvoltage, float& power_mW);
void print_current_voltage_power(float& busvoltage, float& shuntvoltage, float& current_mA, float& loadvoltage, float& power_mW);
void print_current_voltage_power_avg(uint32_t busvoltage, uint32_t shuntvoltage, uint32_t current_mA, uint32_t loadvoltage, uint32_t power_mW);
void printDebugInfo();
#line 14 "z:/Personal/Electronics/particle/i2c_display_bme280_9dof/src/i2c_display_bme280_9dof.ino"
SYSTEM_THREAD(ENABLED);
LSM9DS1 imu;
DS18 sensor(D8);

// VERY IMPORTANT!
//These are the previously determined offsets and scale factors for accelerometer and magnetometer, using MPU9250_cal and Magneto
//The compass will NOT work well or at all if these are not correct

//Accel scale 16457.0 to normalize
float A_B[3]
{ 516.87, -519.49,  125.52};

float A_Ainv[3][3]
  {{  1.04644, -0.00706,  0.03306},
  { -0.00706,  1.01850,  0.00696},
  {  0.03306,  0.00696,  0.98505}};

//  float A_B[3]
//  {  899.26, 2792.96,   26.59};

//  float A_Ainv[3][3]
//   {{  0.97638, -0.09504, -0.01242},
//   { -0.09504,  0.86320, -0.01375},
//   { -0.01242, -0.01375,  1.00225}};

//Mag scale 3746.0 to normalize
float M_B[3]
 {  602.13, 1603.16,-1366.82};

float M_Ainv[3][3]
  {{  1.23900,  0.04296,  0.02386},
  {  0.04296,  1.23621,  0.01496},
  {  0.02386,  0.01496,  1.24947}};

// float M_B[3]
//  {   20.21,  755.67, -230.81};

//  float M_Ainv[3][3]
//   {{ 13.78062,  0.13583, -0.21012},
//   {  0.13583, 14.70536,  0.03224},
//   { -0.21012,  0.03224, 15.57351}};

// local magnetic declination in degrees
float declination = -8.466;

/*
  This tilt-compensated code assumes that the Adafruit LSM9DS1 sensor board is oriented with Y pointing
  to the North, X pointing West, and Z pointing up.
  The code compensates for tilts of up to 90 degrees away from horizontal.
  Facing vector p is the direction of travel and allows reassigning these directions.
  It should be defined as pointing forward,
  parallel to the ground, with coordinates {X, Y, Z} (in magnetometer frame of reference).
*/
float p[] = {0, 1, 0};  //Y marking on sensor board points toward yaw = 0

//SYSTEM_MODE(SEMI_AUTOMATIC);

Adafruit_EEPROM_I2C i2ceeprom;
//Adafruit_FRAM_I2C i2ceeprom;

Adafruit_INA219 ina219;

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);


//define the buttons on the LCD screen.  D2, D3, and D4
#define BUTTON_A  4
#define BUTTON_B  3
#define BUTTON_C  2

/* Set the delay between fresh samples */
#define BNO055_STARTUP_SAMPLE_DELAY_MS (100)
#define BNO055_SAMPLERATE_DELAY_MS (1000)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
// Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/**************************************************************************/
/*
    Wind speed and direction variables
*/
/**************************************************************************/
// int VaneValue;// raw analog value from wind vane
// int Direction;// translated 0 - 360 direction
// int CalDirection;// converted value with offset applied
// int LastValue;
// uint16_t wind_speed_time_interval= 5000; //value in ms
// uint32_t wind_speed_time = 0;
uint8_t vane_pin = A0;
uint8_t vane_switch = D5;
uint8_t wind_pin = D6;
uint8_t calib_light = D7;
volatile unsigned long Rotations = 0; // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine
String heading;
bool page1 = true;

float WindSpeed; // speed miles per hour
uint16_t compass_heading;
uint16_t vane_wind_direction;
uint16_t calibrated_vane_direction;
uint16_t bno_compass_heading;
uint16_t event_compass_heading;
float water_temperature;
float heading_lis3mdl;
float bme_temperature;
float bme_pressure;
float bme_humidity;
float bme_altitude;
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
uint32_t average_time_interval= 300000; //value in ms
uint16_t change_display_time_interval= 2000; //value in ms
uint32_t time_from_last_display = 0;
uint32_t timeFromLastReading = 0;
uint16_t loop_counter = 0;

uint32_t water_temperature_total;
uint32_t bme_temperature_total;
uint32_t bme_pressure_total;
uint32_t bme_humidity_total;
uint32_t bme_altitude_total;
uint32_t busvoltage_total;
uint32_t current_mA_total;
uint32_t power_mW_total;
uint32_t load_voltage_total;
uint32_t shunt_voltage_total;
uint32_t vane_wind_direction_total;
uint32_t heading_total;
uint32_t event_compass_heading_total;
uint32_t bno_compass_heading_total;

uint32_t water_temperature_average;
uint32_t bme_temperature_average;
uint32_t bme_pressure_average;
uint32_t bme_humidity_average;
uint32_t bme_altitude_average;
float busvoltage_average;
uint32_t current_mA_average;
uint32_t load_voltage_average;
uint32_t power_mW_average;
float shunt_voltage_average;
uint32_t vane_wind_direction_average;
uint32_t event_compass_heading_average;
uint32_t bno_compass_heading_average;

#define Offset 0;
//**********************************
//define values for toptechboy compass measurement
float thetaM;
float phiM;
float thetaFold=0;
float thetaFnew;
float phiFold=0;
float phiFnew;
float thetaG=0;
float phiG=0;
float theta;
float phi;
float thetaRad;
float phiRad;
float Xm;
float Ym;
float psi;
float dt;
float Xm_print, Ym_print, Zm_print;
float Xm_off, Ym_off, Zm_off;
float Xm_cal, Ym_cal, Zm_cal;

unsigned long millisOld;
int64_t time_base = 0;
int64_t time_counter  = 60;
/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) {
pinMode(calib_light, OUTPUT);
digitalWrite(calib_light, HIGH);
Wire.begin();
Wire.setClock(400000);

  if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println(F("LSM9DS1 not detected"));
    while (1);
  }


    uint32_t currentFrequency;

  Serial.println("Hello!");
  

  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  ina219.setCalibration_16V_400mA();


time_base = Time.now();
Serial.print("time base value =");
Serial.print(time_base);
Serial.print("\n");

//wind speed and direction setup
//LastValue = 1;

pinMode(vane_pin, INPUT);
pinMode(vane_switch, OUTPUT);

Serial.println("Vane Value\tDirection\tHeading");

pinMode(wind_pin, INPUT);
attachInterrupt(wind_pin, isr_rotation, FALLING);

Serial.println("Davis Wind Speed Test");
Serial.println("Rotations\tMPH");

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
  display.print("BigdaddyAddie weather station!\n");
  display.print("connected!\n");
  display.display(); // actually display all of the above
  delay(1000);

    unsigned status;
    
    // find the BME280
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

   digitalWrite(calib_light, LOW);
    
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop() {
  float Axyz[3], Mxyz[3]; //centered and scaled accel/mag data

  // Update the sensor values whenever new data is available
  if ( imu.accelAvailable() ) imu.readAccel();
  if ( imu.magAvailable() )   imu.readMag();
  
    get_scaled_IMU(Axyz, Mxyz);

    // correct accelerometer handedness
    // Note: the illustration in the LSM9DS1 data sheet implies that the magnetometer 
    // X and Y axes are rotated with respect to the accel/gyro X and Y, but this is not case.
    
    Axyz[0] = -Axyz[0]; //fix accel handedness

//this enables mosfet to turn on wind speed and direction measurement.
digitalWrite(vane_switch, HIGH);

vane_wind_direction = measure_wind_direction();

//WindSpeed = measure_wind_speed();

read_bme_values(bme_temperature, bme_pressure, bme_humidity, bme_altitude);

printValues(bme_temperature, bme_pressure, bme_humidity, bme_altitude);

  bno_compass_heading = get_heading(Axyz, Mxyz, p);

  measure_current_voltage_power(shuntvoltage, busvoltage,current_mA, loadvoltage, power_mW );

  if (sensor.read()) {
    Serial.printf("Temperature %.2f C %.2f F ", sensor.celsius(), sensor.fahrenheit());
    water_temperature = sensor.fahrenheit();
    // Additional info useful while debugging
    printDebugInfo();
  }

  Serial.print("\ncurrent_mA =        ");
  Serial.print( current_mA);
  Serial.print("\n");
  Serial.print("vane wind direction = ");
  Serial.print( vane_wind_direction);
  Serial.print("\n");
  Serial.print("compass heading = ");
  Serial.print( bno_compass_heading);
  Serial.print("\n");
  Serial.print("wind speed rotations = ");
  Serial.print( Rotations);
  Serial.print("\n");
  Serial.print("water tempeature = ");
  Serial.print(water_temperature);
  Serial.print("\n");
  Serial.print("\n");

  print_current_voltage_power(busvoltage, shuntvoltage, current_mA, loadvoltage, power_mW);

water_temperature_total = water_temperature_total + (int) water_temperature;
bme_temperature_total = bme_temperature_total + (int) bme_temperature;
bme_pressure_total = bme_pressure_total + (int) bme_pressure;
bme_humidity_total = bme_humidity_total + (int) bme_humidity;
bme_altitude_total = bme_altitude_total + (int) bme_altitude;
busvoltage_total = busvoltage_total + (int) busvoltage;
load_voltage_total = load_voltage_total + (int) loadvoltage;
shunt_voltage_total = shunt_voltage_total + (int) busvoltage;
current_mA_total = current_mA_total + (int) current_mA;
power_mW_total = power_mW_total + (int) power_mW;
vane_wind_direction_total = vane_wind_direction_total + vane_wind_direction;
//event_compass_heading_total = event_compass_heading_total + event_compass_heading;
bno_compass_heading_total = bno_compass_heading_total + bno_compass_heading;

if ((millis() - timeFromLastReading) > average_time_interval) {

  WindSpeed = (float) Rotations * 2.25 / (float) (millis() - timeFromLastReading) * 1000.;
  water_temperature_average = water_temperature_total / loop_counter;
  bme_temperature_average = bme_temperature_total / loop_counter;
  bme_pressure_average = bme_pressure_total / loop_counter;
  bme_humidity_average = bme_humidity_total / loop_counter;
  bme_altitude_average = bme_altitude_total / loop_counter;
  busvoltage_average = (float) busvoltage_total / (float) loop_counter;
  load_voltage_average = load_voltage_total / loop_counter;
  shunt_voltage_average = (float) shunt_voltage_total / (float) loop_counter;
  current_mA_average = current_mA_total / loop_counter;
  power_mW_average = power_mW_total / loop_counter;
  vane_wind_direction_average = vane_wind_direction_total / loop_counter;
//  event_compass_heading_average = event_compass_heading_total / loop_counter;
  bno_compass_heading_average = bno_compass_heading_total / loop_counter;
  timeFromLastReading = millis();

  print_current_voltage_power_avg(busvoltage_average, shunt_voltage_average, current_mA_average, load_voltage_average, power_mW_average);

  Serial.print("rotations = ");
  Serial.print(Rotations);
  Serial.print("\n");
  Serial.print("\nBME altitude total =        ");
  Serial.print(bme_altitude_total);
  Serial.print("\nBME temp total =            ");
  Serial.print(bme_temperature_total);
  Serial.print("\nloop counter =              ");
  Serial.print(loop_counter);
  Serial.print("\n");
  Serial.print("BME temp average =           ");
  Serial.print(bme_temperature_average);
  Serial.print("\n");
  Serial.print("BME temp average as 8 bit =  ");
  Serial.print((uint8_t) bme_temperature_average);
  Serial.print("\n");
  Serial.print("water temp average as 8 bit =  ");
  Serial.print((uint8_t) water_temperature_average);
  Serial.print("\n");
  Serial.print("current_mA average=         ");
  Serial.print( current_mA_average);
  Serial.print("\n");
  Serial.print("power_mW average =          ");
  Serial.print( power_mW_average);
  Serial.print("\n");

uint8_t w_temp = water_temperature_average;
uint8_t temp = bme_temperature_average;
uint16_t press = bme_pressure_average;
uint8_t humid = bme_humidity_average;
uint8_t ws = WindSpeed;
uint16_t vane_d = vane_wind_direction_average;
//uint16_t event_d = event_compass_heading_average;
uint16_t bno_d = bno_compass_heading_average;
uint8_t c = current_mA_average;
float v = busvoltage_average;
uint16_t p = power_mW_average;

String data = String::format(
"{\"temp\":%d, \"press\":%d, \"humid\":%d, \"wind_speed\":%d, \"wind_direction\":%d, \"bno_direction\":%d, \"w_temp\":%d, \"current\":%d, \"voltage\":%6.3f, \"power\":%d}",
temp, press, humid, ws, vane_d, bno_d, w_temp, c, v, p
);

Serial.print("\ndata\n");
Serial.print(data);
Serial.print("\ndata\n");

Particle.publish("data", data);

  water_temperature_total = 0;
  bme_temperature_total = 0;
  bme_pressure_total = 0;
  bme_humidity_total = 0;
  bme_altitude_total = 0;
  busvoltage_total = 0;
  current_mA_total = 0;
  power_mW_total = 0;
  vane_wind_direction_total = 0;
  bno_compass_heading_total = 0;
  Rotations = 0;  // Set Rotations count to 0 ready for calculations
  Serial.print("loop counter = ");
  Serial.print(loop_counter);
  loop_counter = 0;
  // convert to mph using the formula V=P(2.25/T)
  // V = P(2.25/3) = P * 0.75
  heading = calculateHeading(bno_compass_heading_average);
}

if ((millis() - time_from_last_display) > change_display_time_interval) {
  time_from_last_display = millis();    
  if (page1){
    displayValues1(bme_temperature_average, bme_pressure_average, bme_humidity_average, bme_altitude_average, busvoltage_average, current_mA_average, power_mW_average);
    page1 = false;
  }
  else {
    displayValues2(water_temperature_average, vane_wind_direction_average, heading, WindSpeed);
    page1 = true;
  }
}

loop_counter += 1;

      delay(BNO055_SAMPLERATE_DELAY_MS);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/


void read_bme_values(float& bme_temperature, float& bme_pressure, float& bme_humidity, float& bme_altitude){
bme_temperature = bme.readTemperature()*1.8F + 32.;
bme_pressure = bme.readPressure() / 101325.0F * 760.0F;
bme_humidity = bme.readHumidity();
bme_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
}

void printValues(float& bme_temperature, float& bme_pressure, float& bme_humidity, float& bme_altitude) {
    Serial.print("\n\nTemperature = ");
    Serial.print(bme_temperature);
    Serial.println(" F");

    Serial.print("Pressure = ");

    Serial.print(bme_pressure);
    Serial.println(" mmHg");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme_altitude);
    Serial.println(" m");

    Serial.print("Humidity = ");

    Serial.print(bme_humidity);
    Serial.println(" %");

    Serial.println();
}

void displayValues1(int32_t bme_temperature_average, int32_t bme_pressure_average, int32_t bme_humidity_average, int32_t bme_altitude_average, float busvoltage_average, int32_t current_mA_average, int32_t power_mW_average) {
    display.clearDisplay();
  display.display();
  display.setCursor(0,0);
    display.print("Temp. = ");
    display.print(bme_temperature_average);
    display.println(" F");

    display.print("Press. = ");
    display.print(bme_pressure_average);
    display.println(" mmHg");

    display.print("Humidity = ");
    display.print(bme_humidity_average);
    display.println(" %");

    display.print("Altitude = ");
    display.print(bme_altitude_average);
    display.println(" m");

    display.print("Bus voltage = ");
    display.print(busvoltage_average);
    display.println(" V");

    display.print("current = ");
    display.print(current_mA_average);
    display.println(" mA");

    display.print("power = ");
    display.print(power_mW_average);
    display.println(" mW");

    display.display(); // actually display all of the above
  }
 
void displayValues2(int8_t water_temperature_average, int32_t vane_wind_direction_average, String& heading  , float& WindSpeed) {
    display.clearDisplay();
  display.display();
  display.setCursor(0,0);

    display.print("water temp. = ");
    display.print(water_temperature_average);
    display.println(" F");
    
    display.print("vane dir. = ");
    display.print(vane_wind_direction_average);
    display.println(" d");

    display.print("heading = ");
    display.println(heading);

    display.print("WindSpeed = ");
    display.print(WindSpeed);
    display.println(" mph");

  display.display(); // actually display all of the above
  }

  /**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/

// This is the function that the interrupt calls to increment the rotation count
void isr_rotation () {
//a debounce time of 22 ms is equivalent to wind of 100 mph.
if ((millis() - ContactBounceTime) > 22 ) { // debounce the switch contact.
Rotations++;
ContactBounceTime = millis();
}

}

int get_heading(float acc[3], float mag[3], float p[3])
{
  float W[3], N[3]; //derived direction vectors

  // cross "Up" (acceleration vector, g) with magnetic vector (magnetic north + inclination) with  to produce "West"
  vector_cross(acc, mag, W);
  vector_normalize(W);

  // cross "West" with "Up" to produce "North" (parallel to the ground)
  vector_cross(W, acc, N);
  vector_normalize(N);

  // compute heading in horizontal plane, correct for local magnetic declination
  
  int heading = round(atan2(vector_dot(W, p), vector_dot(N, p)) * 180 / M_PI + declination);
  heading = -heading; //conventional nav, heading increases North to East
  heading = (heading + 720)%360; //apply compass wrap
  return heading;
}

// subtract offsets and correction matrix to accel and mag data

void get_scaled_IMU(float Axyz[3], float Mxyz[3]) {
  byte i;
  float temp[3];
    Axyz[0] = imu.ax;
    Axyz[1] = imu.ay;
    Axyz[2] = imu.az;
    Mxyz[0] = imu.mx;
    Mxyz[1] = imu.my;
    Mxyz[2] = imu.mz;
  //apply offsets (bias) and scale factors from Magneto
  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  //apply offsets (bias) and scale factors from Magneto
  for (int i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

// basic vector operations
void vector_cross(float a[3], float b[3], float out[3])
{
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

uint16_t measure_wind_direction(){

uint16_t VaneValue;// raw analog value from wind vane
uint16_t Direction;// translated 0 - 360 direction
uint16_t CalDirection;// converted value with offset applied
//int LastValue = 0;
VaneValue = analogRead(vane_pin);
//digitalWrite(vane_switch, LOW);
Serial.println("analog read value = ");
Serial.print(VaneValue);
Direction = map(VaneValue, 0, 4095, 0, 360);
CalDirection = Direction + Offset;

if(CalDirection > 360)
CalDirection = CalDirection - 360;

if(CalDirection < 0)
CalDirection = CalDirection + 360;

return CalDirection;
}

String calculateHeading(int direction) {
  String heading = "xx";
if(direction < 22)
heading = "N";
else if (direction < 67)
heading = "NE";
else if (direction < 112)
heading = "E";
else if (direction < 157)
heading = "SE";
else if (direction < 212)
heading = "S";
else if (direction < 247)
heading = "SW";
else if (direction < 292)
heading = "W";
else if (direction < 337)
heading = "NW";
else
heading = "N";

return heading;
} 


void measure_current_voltage_power(float& shuntvoltage, float& busvoltage, float& current_mA, float& loadvoltage, float& power_mW){


  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

}
void print_current_voltage_power(float& busvoltage, float& shuntvoltage, float& current_mA, float& loadvoltage, float& power_mW){

  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");
}
void print_current_voltage_power_avg(uint32_t busvoltage, uint32_t shuntvoltage, uint32_t current_mA, uint32_t loadvoltage, uint32_t power_mW){

  
  Serial.print("Bus Voltage average:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage average: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage average:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current average:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power average:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");
}

void printDebugInfo() {
  // If there's an electrical error on the 1-Wire bus you'll get a CRC error
  // Just ignore the temperature measurement and try again
  if (sensor.crcError()) {
    Serial.print("CRC Error ");
  }

  // Print the sensor type
  const char *type;
  switch(sensor.type()) {
    case WIRE_DS1820: type = "DS1820"; break;
    case WIRE_DS18B20: type = "DS18B20"; break;
    case WIRE_DS1822: type = "DS1822"; break;
    case WIRE_DS2438: type = "DS2438"; break;
    default: type = "UNKNOWN"; break;
  }
  Serial.print(type);

  // Print the ROM (sensor type and unique ID)
  uint8_t addr[8];
  sensor.addr(addr);
  Serial.printf(
    " ROM=%02X%02X%02X%02X%02X%02X%02X%02X",
    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]
  );

  // Print the raw sensor data
  uint8_t data[9];
  sensor.data(data);
  Serial.printf(
    " data=%02X%02X%02X%02X%02X%02X%02X%02X%02X",
    data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]
  );
}


// uint16_t get_event_compass_heading(){
// sensors_event_t event;
//   bno.getEvent(&event);
  
//   /* Display the floating point data */
//   Serial.print("X: ");
//   Serial.print(event.orientation.x, 4);
//   Serial.print("\tY: ");
//   Serial.print(event.orientation.y, 4);
//   Serial.print("\tZ: ");
//   Serial.print(event.orientation.z, 4);
//   Serial.println("");

//   Serial.println("\n\n");

//   compass_heading = (int)(event.orientation.x + 0);

//   if(compass_heading > 360) {
//     compass_heading = compass_heading - 360;
//   }
  
//   Serial.print("compass heading:  ");
//   Serial.print(compass_heading, 4);
//   return compass_heading;
// }