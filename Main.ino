//#define Wire Wire1
// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7

// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

// For the sensor of orientation
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
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
   Connect SCL to SCL pin (analog 5 on Arduino UNO)
   Connect SDA to SDA pin (analog 4 on Arduino UNO)
   Connect VDD to 3-5V DC (depending on your board's logic level)
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

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

/*****************************************************************************/
//  Function:  Get the accelemeter of the x/y/z axis.
//  Hardware:  Grove - 3-Axis Analog Accelerometer
//  Arduino IDE: Arduino-1.0
//  Author:  Frankie.Chu
//  Date:    Jan 11,2013
//  Version: v1.0
//  by www.seeedstudio.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
/*******************************************************************************/

#include "ADXL335.h"
ADXL335 accelerometer;
void setup()
{
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  delay(5000);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  // for the sensor of orientation
  //  Serial.begin(115200);
  //  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//    while (1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);

  delay(1000);
  //  analogReference(EXTERNAL);
  accelerometer.begin();
}

// ################################################################################### //

class SimpleKalman {
  private:
    double _time1;
    double _time2;

    double _initial_pos;
    double _lastPosition;
    double _lastVelocity;
    double _position;
    double _velocity;
    double _estimatedPosition;
    double _estimatedVelocity;
    double _heading;

    float _processErrorQ;
    float _sensorErrorR;
    float _translationH;

    float _P[2][2]; //error covariance matrix;
  public:
    SimpleKalman();

    float GPStokm(float lon1, float lon2, float lat1, float lat2);
    float mpermillis_to_ms(float m_millis);

    void setPosition(double pos);
    void setVelocity(double velocity);
    void setHeading(double heading);

    void setEstimatedCovariance(float cov);
    void setProcessError(float err_q);
    void setSensorError(float err_r);
    void setTranslation(float h);

    void predictEstimate(double u, double t1);
    void updateEstimate(double pos, double t2);

    double getPosition();
    double getVelocity();
    double getHeading();
};

#define EARTH_RADIUS 6368.061

//Constructor -------------------
SimpleKalman::SimpleKalman() {
  _lastPosition = 0;
  _lastVelocity = 0;
  _position = 0;
  _velocity = 0;
  _estimatedPosition = 0;
  _estimatedVelocity = 0;
  _heading = 0;

  // Covariance, Variance, and Error estimates
  //are preset to specifications roughly suitable for my project,
  //tuning through "Setters" is encourages
  _processErrorQ = 0.03;
  _sensorErrorR = 0.1;
  _translationH = 1;

  _P[0][0] = 0.5;
  _P[0][1] = 0;
  _P[1][0] = 0;
  _P[1][1] = 0.5;
};

//Util ---------------------

//return a delta distance in meters from two coordinates
float SimpleKalman::GPStokm(float lon1, float lon2, float lat1, float lat2) {
  float dlon = lon2 - lon1;
  float dlat = lat2 - lat1;
  float a = square(sin(dlat * 0.5)) + cos(lat1) * cos(lat2) * square(sin(dlon * 0.5));
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float d = EARTH_RADIUS * c / 1000;
  return d;
};

float SimpleKalman::mpermillis_to_ms(float m_millis) {
  float ms = m_millis * 1000;
};


//Setters ---------------------

void SimpleKalman::setPosition(double pos) {
  this->_initial_pos = pos;

  //this->_position = pos;
};

void SimpleKalman::setVelocity(double velocity) {
  this->_velocity = velocity;
};

void SimpleKalman::setHeading(double heading) {
  this->_heading = heading;
};

void SimpleKalman::setEstimatedCovariance(float cov) {
  this->_P[0][0] = cov;
  this->_P[1][1] = cov;
};

void SimpleKalman::setProcessError(float err_q) {
  this->_processErrorQ = err_q;
};

void SimpleKalman::setSensorError(float err_r) {
  this->_sensorErrorR = err_r;
};

void SimpleKalman::setTranslation(float h) {
  this->_translationH = h;
};


//Getters ----------------------------

double SimpleKalman::getPosition() {
  return this->_estimatedPosition;
};

double SimpleKalman::getVelocity() {
  return this->_estimatedVelocity;
};

double SimpleKalman::getHeading() {
  return this->_heading;
};


//Calculations ---------------------

void SimpleKalman::predictEstimate(double u, double t1) {
  _time1 = t1; //time of accelerometer sample
  double dt = _time2 - _time1; //time since previous location update sample

  _lastPosition = _position;
  _lastVelocity = _velocity;
  //Predict xhat
  _position = _lastPosition + _lastVelocity * dt + 0.5 * dt * dt * u;
  _velocity = _lastVelocity + dt * u;

  //Update Covariance Matrix
  // Pn = A(Pn-1)AT + Q
  //[1 dt][p00 p01][1  0]
  //[0 1 ][p10 p11][dt 1]
  _P[0][0] = _P[0][0] + _P[1][0] * dt + _P[0][1] * dt + _P[1][1] * dt * dt + _processErrorQ;
  _P[0][1] = _P[1][1] * dt + _P[0][1];
  _P[1][0] = _P[1][0] + _P[1][1] * dt;
  _P[1][1] = _P[1][1] + _processErrorQ;
};


void SimpleKalman::updateEstimate(double pos, double t2) {

  //input processing step
  double dt = t2 - _time2; //Delta T from location sample before this
  _time2 = t2; //set new location sample timestamp

  double velocity = (_estimatedPosition - pos) / dt; //calculate avg V

  //Observation Step --------------------------
  double _positionInnovation = pos - _position;
  double _velocityInnovation = velocity - _velocity;

  double _S[2][2] = {{0, 0}, {0, 0}};
  _S[0][0] = _P[0][0] + _sensorErrorR;
  _S[0][1] = _P[0][1];
  _S[1][0] = _P[1][0];
  _S[1][1] = _P[1][1] + _sensorErrorR;

  //Kalman Gain
  double _K[2][2] = {{0, 0}, {0, 0}};
  //(P)(S^-1) expanded
  _K[0][0] = (_P[0][0] * _S[1][1] - _P[0][1] * _S[1][0] ) * ( 1 / ( (_S[0][0] * _S[1][1]) - (_S[0][1] * _S[1][0]) ) );
  _K[0][1] = (-1 * _P[0][0] * _S[0][1] + _P[0][1] * _S[0][0] ) * ( 1 / ( (_S[0][0] * _S[1][1]) - (_S[0][1] * _S[1][0]) ) );
  _K[1][0] = (_P[1][0] * _S[1][1] - _P[1][1] * _S[1][0] ) * ( 1 / ( (_S[0][0] * _S[1][1]) - (_S[0][1] * _S[1][0]) ) );
  _K[1][1] = (-1 * _P[1][0] * _S[0][1] + _P[1][1] * _S[0][0] ) * ( 1 / ( (_S[0][0] * _S[1][1]) - (_S[0][1] * _S[1][0]) ) );

  //State Update
  _estimatedPosition = _position + (_K[0][0] * _positionInnovation + _K[0][1] * _velocityInnovation);
  _estimatedVelocity = _velocity + (_K[1][0] * _positionInnovation + _K[1][1] * _velocityInnovation);
};

// ################################################################################### //
float lon1;
float lon2;
float lat1;
float lat2;
float distance;
uint32_t timer = millis();
//char c;
SimpleKalman One;
void loop()                     // run over and over again
{
  // for the sensor of orientation
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  // for the GPS
  char c = GPS.read();

  //  Serial.print("\nRead the data of the GPS");
  c = GPS.read();
  c = GPS.read();
  c = GPS.read();
  c = GPS.read();
  c = GPS.read();
  c = GPS.read();
  c = GPS.read();
  c = GPS.read();
  c = GPS.read();

  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  /*
    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis())  timer = millis();

    // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer > 2000) {
      timer = millis(); // reset the timer

      Serial.print("\nTime: ");
      if (GPS.hour < 10) { Serial.print('0'); }
      Serial.print(GPS.hour, DEC); Serial.print(':');
      if (GPS.minute < 10) { Serial.print('0'); }
      Serial.print(GPS.minute, DEC); Serial.print(':');
      if (GPS.seconds < 10) { Serial.print('0'); }
      Serial.print(GPS.seconds, DEC); Serial.print('.');
      if (GPS.milliseconds < 10) {
        Serial.print("00");
      } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
        Serial.print("0");
      }
      Serial.println(GPS.milliseconds);
      Serial.print("Date: ");
      Serial.print(GPS.day, DEC); Serial.print('/');
      Serial.print(GPS.month, DEC); Serial.print("/20");
      Serial.println(GPS.year, DEC);
      Serial.print("Fix: "); Serial.print((int)GPS.fix);
      Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
      if (GPS.fix) {
        Serial.print("Location: ");
        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        Serial.print(", ");
        Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Angle: "); Serial.println(GPS.angle);
        Serial.print("Altitude: "); Serial.println(GPS.altitude);
        Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      }
    }

    // Print the estimated position with the heading angle
    Serial.println("");
    Serial.println("position of Y/Z: ");
    Serial.print(One.getPosition());
    Serial.println("");
    Serial.println("heading of X: ");
    Serial.print(One.getHeading());
  */
  /* Display the floating point data */
  Serial.println("");
  Serial.print("Fix: "); Serial.print((int)GPS.fix);
  /*
    Serial.print("\tX: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);
  */
  Serial.println("");

  // Set the heading angle with the distance of travel
  One.setHeading(event.orientation.x);
  if (GPS.fix) {
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

    lon1 = lon2;
    lat1 = lat2;
    lon2 = GPS.lon;
    lat2 = GPS.lat;

    // Set the distance of travel
    distance = One.GPStokm(lon1, lon2, lat1, lat2) * 1000;
    Serial.print("Distance: "); Serial.println(distance);
    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    Serial.print("Angle: "); Serial.println(GPS.angle);
    Serial.print("Altitude: "); Serial.println(GPS.altitude);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);

    if (distance > 0)
      // Update with the position of travel of the GPS
      One.updateEstimate(One.getPosition() + distance, timer);
  }

  //  Serial.print("\nRead the data of the GPS");
  c = GPS.read();
  c = GPS.read();
  c = GPS.read();
  c = GPS.read();
  c = GPS.read();
  c = GPS.read();
  c = GPS.read();
  c = GPS.read();
  c = GPS.read();

  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // for the sensor of orientation
  /* Optional: Display calibration status */
  //  displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //  displaySensorStatus();

  /* New line for the next sample */
  Serial.println("");
  //  Serial.print("\n");

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);

  // for the accelerometer
  int x, y, z;
  accelerometer.getXYZ(&x, &y, &z);
  Serial.println("value of X/Y/Z: ");
  Serial.println(x);
  Serial.println(y);
  Serial.println(z);
  float ax, ay, az;
  accelerometer.getAcceleration(&ax, &ay, &az);
  Serial.println("acceleration of X/Y/Z: ");
  Serial.print(ax);
  Serial.println(" g");
  Serial.print(ay);
  Serial.println(" g");
  Serial.print(az);
  Serial.println(" g");
  //  delay(500);

  /* New line for the next sample */
  Serial.println("");
  //  Serial.print("\n");

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);

  // Estimate the next position with the acceleration
  One.predictEstimate(sqrt((ay - 5.02) * 9.81 * (ay - 5.02) * 9.81 + (az - 4.84) * 9.81 * (az - 4.84) * 9.81), timer);

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
