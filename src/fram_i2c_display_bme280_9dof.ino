#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <imumaths.h>
#include <SPI.h>
#include <Adafruit_INA219.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_BME280.h>
#include "Adafruit_EEPROM_I2C.h"
#include "Adafruit_FRAM_I2C.h"
#include "math.h"

Adafruit_EEPROM_I2C i2ceeprom;
//Adafruit_FRAM_I2C i2ceeprom;

Adafruit_INA219 ina219;

#define EEPROM_ADDR 0x50  // the default address for the I2C FRAM eeprom

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
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

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
float bme_temperature;
float bme_pressure;
float bme_humidity;
float bme_altitude;
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
uint16_t average_time_interval= 300000; //value in ms
uint16_t change_display_time_interval= 2000; //value in ms
uint32_t time_from_last_display = 0;
uint32_t timeFromLastReading = 0;
uint16_t loop_counter = 0;

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

uint32_t bme_temperature_average;
uint32_t bme_pressure_average;
uint32_t bme_humidity_average;
uint32_t bme_altitude_average;
uint32_t busvoltage_average;
uint32_t current_mA_average;
uint32_t load_voltage_average;
uint32_t power_mW_average;
uint32_t shunt_voltage_average;
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

unsigned long millisOld;
int64_t time_base = 0;
int64_t time_counter  = 60;
/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) {

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
  display.print("BigdaddyAddie weather station!\n");
  display.print("connected!\n");
  display.display(); // actually display all of the above

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

  //add a test for fram
if (i2ceeprom.begin(0x50)) {  // you can stick the new i2c addr in here, e.g. begin(0x51);
    Serial.println("Found I2C EEPROM");
    display.clearDisplay();
    display.println("Found I2C FRAM");
    display.display();
  } else {
    Serial.println("I2C EEPROM not identified ... check your connections?\r\n");
    while (1) delay(10);
  }
  
  // // Write the first byte to 0xAF
  // uint8_t test = 0xAF;
  // i2ceeprom.write(0x0, test);


int eeAddress = 0;
//eeprom_test();

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   Serial.print("time base value =");
Serial.print(time_base);
Serial.print("\n");
  delay(1000);
   
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
millisOld=millis();
    sensors_event_t event;
    bno.getEvent(&event);
    /* always recal the mag as It goes out of calibration very often */
    if (foundCalib){
        Serial.println("Move sensor slightly to calibrate magnetometers");
        display.clearDisplay();
        display.println("Move sensor slightly");
        display.display();
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            displayCalStatus();
            delay(BNO055_STARTUP_SAMPLE_DELAY_MS);
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
            delay(BNO055_STARTUP_SAMPLE_DELAY_MS);
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
  
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) {

//this enables mosfet to turn on wind speed and direction measurement.
digitalWrite(vane_switch, HIGH);

vane_wind_direction = measure_wind_direction();

//WindSpeed = measure_wind_speed();

read_bme_values(bme_temperature, bme_pressure, bme_humidity, bme_altitude);

printValues(bme_temperature, bme_pressure, bme_humidity, bme_altitude);



  // delay(delayTime);
  // if(!digitalRead(BUTTON_A)) display.print("A");
  // if(!digitalRead(BUTTON_B)) display.print("B");
  // if(!digitalRead(BUTTON_C)) display.print("C");
  // delay(10);
  // yield();
  // display.display();

event_compass_heading = get_event_compass_heading();

  bno_compass_heading = get_compass_heading();

  measure_current_voltage_power(shuntvoltage, busvoltage,current_mA, loadvoltage, power_mW );

  Serial.print("\ncurrent_mA =        ");
  Serial.print( current_mA);
  Serial.print("\n");
  Serial.print("vane wind direction = ");
  Serial.print( vane_wind_direction);
  Serial.print("\n");

  print_current_voltage_power(busvoltage, shuntvoltage, current_mA, loadvoltage, power_mW);

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
event_compass_heading_total = event_compass_heading_total + event_compass_heading;
bno_compass_heading_total = bno_compass_heading_total + bno_compass_heading;

if ((millis() - timeFromLastReading) > average_time_interval) {

  WindSpeed = (float) Rotations * 2.25 / (float) (millis() - timeFromLastReading) * 1000.;
  bme_temperature_average = bme_temperature_total / loop_counter;
  bme_pressure_average = bme_pressure_total / loop_counter;
  bme_humidity_average = bme_humidity_total / loop_counter;
  bme_altitude_average = bme_altitude_total / loop_counter;
  busvoltage_average = busvoltage_total / loop_counter;
  load_voltage_average = load_voltage_total / loop_counter;
  shunt_voltage_average = shunt_voltage_total / loop_counter;
  current_mA_average = current_mA_total / loop_counter;
  power_mW_average = power_mW_total / loop_counter;
  vane_wind_direction_average = vane_wind_direction_total / loop_counter;
  event_compass_heading_average = event_compass_heading_total / loop_counter;
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
  Serial.print("current_mA average=         ");
  Serial.print( current_mA_average);
  Serial.print("\n");
  Serial.print("power_mW average =          ");
  Serial.print( power_mW_average);
  Serial.print("\n");

uint8_t temp = bme_temperature_average;
uint16_t press = bme_pressure_average;
uint8_t humid = bme_humidity_average;
uint8_t ws = WindSpeed;
uint16_t vane_d = vane_wind_direction_average;
uint16_t event_d = event_compass_heading_average;
uint16_t bno_d = bno_compass_heading_average;
uint8_t c = current_mA_average;
uint8_t v = busvoltage_average;
uint16_t p = power_mW_average;

String data = String::format(
"{\"temp\":%d, \"press\":%d, \"humid\":%d, \"ws\":%d, \"vane_d\":%d, \"event_d\":%d, \"bno_d\":%d, \"c\":%d, \"v\":%d, \"p\":%d}",
temp, press, humid, ws, vane_d, event_d, bno_d, c, v, p
);

Serial.print("\ndata\n");
Serial.print(data);
Serial.print("\ndata\n");

Particle.publish("data", data);

  bme_temperature_total = 0;
  bme_pressure_total = 0;
  bme_humidity_total = 0;
  bme_altitude_total = 0;
  busvoltage_total = 0;
  current_mA_total = 0;
  power_mW_total = 0;
  vane_wind_direction_total = 0;
  event_compass_heading_total = 0;
  bno_compass_heading_total = 0;
  Rotations = 0;  // Set Rotations count to 0 ready for calculations
  Serial.print("loop counter = ");
  Serial.print(loop_counter);
  loop_counter = 0;
  // convert to mph using the formula V=P(2.25/T)
  // V = P(2.25/3) = P * 0.75
  heading = calculateHeading(int((event_compass_heading_average + bno_compass_heading_average) / 2));
}

if ((millis() - time_from_last_display) > change_display_time_interval) {
  time_from_last_display = millis();    
  if (page1){
    displayValues1(bme_temperature_average, bme_pressure_average, bme_humidity_average, bme_altitude_average, busvoltage_average, current_mA_average, power_mW_average);
    page1 = false;
  }
  else {
    displayValues2(vane_wind_direction_average, heading, WindSpeed);
    page1 = true;
  }
}
  //print_current_voltage_power(shuntvoltage, busvoltage,current_mA, loadvoltage, power_mW );

  //Particle.publish("office temperature", String(bme_temperature));


    //   adafruit_bno055_offsets_t newCalib;
    // bno.getSensorOffsets(newCalib);
    // displaySensorOffsets(newCalib);

loop_counter += 1;

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
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Sys:");
    display.print(system, DEC);
    display.print(" G:");
    display.print(gyro, DEC);
    display.print(" A:");
    display.print(accel, DEC);
    display.print(" M:");
    display.print(mag, DEC);
    display.display();
}

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

void displayValues1(int32_t bme_temperature_average, int32_t bme_pressure_average, int32_t bme_humidity_average, int32_t bme_altitude_average, int32_t busvoltage_average, int32_t current_mA_average, int32_t power_mW_average) {
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
 
void displayValues2(int32_t vane_wind_direction_average, String& heading  , float& WindSpeed) {
    display.clearDisplay();
  display.display();
  display.setCursor(0,0);

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


// This is the function that the interrupt calls to increment the rotation count
void isr_rotation () {
//a debounce time of 22 ms is equivalent to wind of 100 mph.
if ((millis() - ContactBounceTime) > 22 ) { // debounce the switch contact.
Rotations++;
ContactBounceTime = millis();
}

}

uint16_t get_compass_heading() {
  // put your main code here, to run repeatedly:
uint8_t system, gyro, accel, mg = 0;
bno.getCalibration(&system, &gyro, &accel, &mg);
imu::Vector<3> acc =bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
imu::Vector<3> gyr =bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
imu::Vector<3> mag =bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
//measured value of tilt in x
thetaM=-atan2(acc.x()/9.8,acc.z()/9.8)/2/3.141592654*360;
//measured value of tilt in y
phiM=-atan2(acc.y()/9.8,acc.z()/9.8)/2/3.141592654*360;
//filtered value for tilt in y
phiFnew=.95*phiFold+.05*phiM;
//filtered value for tilt in x
thetaFnew=.95*thetaFold+.05*thetaM;
 //this is the time through one loop of program
dt=(millis()-millisOld)/1000.;
millisOld=millis();
//measures tilt in x with filter to remove vibration but still quick to respond.  lesson 9
//complimentary filter
theta=(theta+gyr.y()*dt)*.95+thetaM*.05;
//measures tilt in y with filter to remove vibration but still quick to respond.  lesson 9
phi=(phi-gyr.x()*dt)*.95+ phiM*.05;

thetaG=thetaG+gyr.y()*dt;
phiG=phiG-gyr.x()*dt;
 
phiRad=phi/360*(2*3.14);
thetaRad=theta/360*(2*3.14);
 
 //tilt compensated x
Xm=mag.x()*cos(thetaRad)-mag.y()*sin(phiRad)*sin(thetaRad)+mag.z()*cos(phiRad)*sin(thetaRad);
//tilt compensated y
Ym=mag.y()*cos(phiRad)+mag.z()*sin(phiRad);
 //compass heading
psi=atan2(Ym,Xm)/(2*3.14)*360;
 
// Serial.print(acc.x()/9.8);
// Serial.print(",");
// Serial.print(acc.y()/9.8);
// Serial.print(",");
// Serial.print(acc.z()/9.8);
// Serial.print(",");
// Serial.print(accel);
// Serial.print(",");
// Serial.print(gyro);
// Serial.print(",");
// Serial.print(mg);
// Serial.print(",");
// Serial.print(system);
// Serial.print(",");
// Serial.print(thetaM);
// Serial.print(",");
// Serial.print(phiM);
// Serial.print(",");
// Serial.print(thetaFnew);
// Serial.print(",");
// Serial.print(phiFnew);
// Serial.print(",");
// Serial.print(thetaG);
// Serial.print(",");
// Serial.print(phiG);
// Serial.print(",");
// Serial.print(theta);
// Serial.print(",");
// Serial.print(phi);
// Serial.print(",");
// Serial.println(psi);
 
phiFold=phiFnew;
thetaFold=thetaFnew;
 psi = psi + 180;
 if(psi >= 360) {
   psi = psi -360;
 }
 return((int)psi);
//delay(BNO055_SAMPLERATE_DELAY_MS);
}

void eeprom_test(){
  uint32_t max_addr;
  //variables for calibration read from memory
int test = 55;
      // Try to determine the size by writing a value and seeing if it changes the first byte
  Serial.println("Testing size!");
  for (max_addr = 1; max_addr < 0x7FFF; max_addr++) {
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
}

// float measure_wind_speed() {
// uint16_t wind_speed_time_interval= 10000; //value in ms
// uint32_t wind_speed_time = 0;
// float WindSpeed = 0;
// if ((millis() - wind_speed_time) > wind_speed_time_interval) {
// // Only update the display if change greater than 2 degrees.
//   wind_speed_time = millis();
//   WindSpeed = Rotations * 2.25 / (millis() - wind_speed_time) * 1000;
// Rotations = 0;  // Set Rotations count to 0 ready for calculations
// // convert to mph using the formula V=P(2.25/T)
// // V = P(2.25/3) = P * 0.75
// }
// return WindSpeed;
// }

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

//delay(100);

  // if(abs(CalDirection - LastValue) > 5)
  // {
  //   Serial.print("Vanevalue -----------------\n");
  // Serial.print(VaneValue); Serial.print("\t\t");
  // Serial.print(CalDirection); Serial.print("\t\t");
  // getHeading(CalDirection);
  // LastValue = CalDirection;
  // }

return CalDirection;
}

// Converts compass direction to heading
void printHeading(int direction) {
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

void print_heading_pitch_roll() {
      /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
  /* The WebSerial 3D Model Viewer expects data as heading, pitch, roll */
  Serial.print(F("Heading, pitch, roll: "));
  Serial.print((float)event.orientation.x);
  Serial.print(F(", "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(", "));
  Serial.print((float)event.orientation.z);
  Serial.println(F(""));
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


uint16_t get_event_compass_heading(){
sensors_event_t event;
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

  compass_heading = (int)(event.orientation.x + 0);

  if(compass_heading > 360) {
    compass_heading = compass_heading - 360;
  }
  
  Serial.print("compass heading:  ");
  Serial.print(compass_heading, 4);
  return compass_heading;
}


//removed from line 364
  // /* The WebSerial 3D Model Viewer also expects data as roll, pitch, heading */
  // imu::Quaternion quat = bno.getQuat();
  
  // Serial.print(F("Quaternion: "));
  // Serial.print((float)quat.w());
  // Serial.print(F(", "));
  // Serial.print((float)quat.x());
  // Serial.print(F(", "));
  // Serial.print((float)quat.y());
  // Serial.print(F(", "));
  // Serial.print((float)quat.z());
  // Serial.println(F(""));

  /* Also send calibration data for each sensor. */