#include <Arduino.h>
#include <MPU9250.h>
#include <Wire.h>

#include <sstream>
#include <string>

#include "EEPROM.h"
#include "PositionSensor.h"
#include "SPI.h"
#include "bike_light_util.h"

#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_INT_PIN 19

float window_size = 20;

using namespace BikeLight;

PositionSensor pos_sensor;

/* EEPROM buffer to mag bias and scale factors */
uint8_t eeprom_buffer[24];
float value;

void setup() {
  Serial.begin(115200);

  pos_sensor.begin();

  // Wire.begin(SDA_PIN, SCL_PIN);

  // Wire.setClock(400000);
  // Serial.println("begining bme280");
  // if (!bme.begin(BME280_ADDR, &Wire)) {
  //   Serial.println(
  //       "Could not find a valid BME280 sensor, check wiring, address, "
  //       "sensorID!");
  //   Serial.print("SensorID was: 0x");
  //   Serial.print(
  //       "        ID of 0xFF probably means a bad address, a BMP 180 or BMP "
  //       "085\n");
  //   Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
  //   Serial.print("        ID of 0x60 represents a BME 280.\n");
  //   Serial.print("        ID of 0x61 represents a BME 680.\n");
  // }
  // Serial.println("done bme280");
}

void loop() {
  auto heading = pos_sensor.read_direction(true);
  Serial.println(heading.c_str());
  Serial.println();
  Serial.println();

  delay(1000);
}

// CalibratedData:
// 8 fig vertical and hoz: bX=39.0, bY=52.0, bZ=18.1, sX=1.0, sY=0.8, sZ=1.4,
// 8 fig stop before     : bX=46.5, bY=46.8, bZ=39.6, sX=1.0, sY=0.9, sZ=1

// Sparkfun:
// Accel: 0.14, -0.86, 0.76 g
// Gyro: -2.74, -1.10, 1.04 dps
// Mag: 71.57, 50.41, 34.96 uT
// Borderflight:
// ], Mag (uT) [  00073.24,  00051.40,  00034.79 ]
// Borderflight calibreated:
// Mag (uT) [  00039.47,  00007.19,  00027.02 ]
//
// No-accelerometer heading:
// 86 EAST
// yaw_rad: -1.5, yaw_deg: -87.0
// , heading_rad: 4.8, heading_deg:273.0
// , filtered_heading_rad:0.2, filtered_heading_deg: 13.6
//
// accelerometer heading:
//
// 88 : EAST
// yaw_rad: -1.8, yaw_deg: -102.3
// , heading_rad: 4.5, heading_deg:257.7
// , filtered_heading_rad:3.3, filtered_heading_deg: 189.8
//
// 144 : SOUTH-EAST
// yaw_rad: -2.5, yaw_deg: -140.6
// , heading_rad: 3.8, heading_deg:219.4
// , filtered_heading_rad:4.0, filtered_heading_deg: 229.2

float filtered_heading_rad;
String get_heading(float hx, float hy, float hz, float ax, float ay, float az) {
  /* Normalize accelerometer and magnetometer data */
  float a = sqrtf(ax * ax + ay * ay + az * az);
  ax /= a;
  ay /= a;
  az /= a;

  float h = sqrtf(hx * hx + hy * hy + hz * hz);
  hx /= h;
  hy /= h;
  hz /= h;

  /* Compute euler angles */
  float pitch_rad = asinf(ax);
  float roll_rad = asinf(-ay / cosf(pitch_rad));
  float yaw_rad =
      atan2f(hz * sinf(roll_rad) - hy * cosf(roll_rad),
             hx * cosf(pitch_rad) + hy * sinf(pitch_rad) * sinf(roll_rad) +
                 hz * sinf(pitch_rad) * cosf(roll_rad));
  float heading_rad = constrainAngle360(yaw_rad);
  /* Filtering heading */
  filtered_heading_rad =
      (filtered_heading_rad * (window_size - 1.0f) + heading_rad) / window_size;

  /* Display the results */
  std::stringstream ss = get_fixed_stringstream();

  ss << "yaw_rad: " << yaw_rad << ", yaw_deg: " << yaw_rad * RAD_TO_DEG
     << std::endl
     << ", heading_rad: " << heading_rad
     << ", heading_deg:" << heading_rad * RAD_TO_DEG << std::endl
     << ", filtered_heading_rad:" << filtered_heading_rad
     << ", filtered_heading_deg: " << filtered_heading_rad * RAD_TO_DEG
     << std::endl;
  Serial.println(ss.str().c_str());
  return "";
}

/* Bound angle between 0 and 360 */
float constrainAngle360(float dta) {
  dta = fmod(dta, 2.0 * PI);
  if (dta < 0.0) dta += 2.0 * PI;
  return dta;
}

String get_direction(float x, float y, float z) {
  int heading = 0;

  if (y > 0) {
    heading = 90 - atan(x / y) * (180 / M_PI);
  } else if (y < 0) {
    heading = 270 - atan(x / y) * (180 / M_PI);
  } else if (y == 0 && x < 0) {
    heading = 180;
  } else if (y == 0 && x > 0) {
    heading = 0;
  }
  auto direction = "";
  if (heading > 338 || heading < 22) {
    direction = "NORTH";
  } else if (heading > 22 && heading < 68) {
    direction = "NORTH-EAST";
  } else if (heading > 68 && heading < 113) {
    direction = "EAST";
  } else if (heading > 113 && heading < 158) {
    direction = "SOUTH-EAST";
  } else if (heading > 158 && heading < 203) {
    direction = "SOUTH";
  } else if (heading > 203 && heading < 248) {
    direction = "SOUTH-WEST";
  } else if (heading > 248 && heading < 293) {
    direction = "WEST";
  } else if (heading > 293 && heading < 338) {
    direction = "NORTH-WEST";
  } else {
    direction = "dont know";
  }
  return String(heading) + " : " + direction;
}

// Note:
// North: Scaled. Acc (mg) [ -00069.15, -00276.00,  00016.50 ], Tmp (C) [  ]
//         Mag (uT) [ -00028.05, -00051.00,  00032.40 ]
//         Mag (uT) [  00010.35,  00046.65,  00024.90 ]

// Sparkfun lib:
//  ], Mag (uT) [  00038.10,  00001.35,  00029.85 ]
// 2 : NORTH
//  ], Mag (uT) [  00038.55,  00001.35,  00028.05 ]
// 2 : NORTH

//  ], Mag (uT) [  00035.55,  -00002.70,  00024.58 ]
// 353 : NORTH
