#include <Arduino.h>
#include <MPU9250.h>
#include <Wire.h>

#include <sstream>
#include <string>

#include "EEPROM.h"
#include "SPI.h"

#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_INT_PIN 19

float window_size = 20;

// 8 fig vertical and hoz: bX=39.0, bY=52.0, bZ=18.1, sX=1.0, sY=0.8, sZ=1.4,
// bias X, Scale X, bias Y, scale Y...
std::vector<float> CalData = {39.0, 1.0, 52.0, 0.8, 18.1, 1.4};

MPU9250 imu(Wire, 0x68);

/* EEPROM buffer to mag bias and scale factors */
uint8_t eeprom_buffer[24];
float value;

void setup() {
  Serial.begin(115200);

  // start communication with IMU
  while (1) {
    int status = imu.begin();
    if (status < 0) {
      Serial.println("IMU initialization unsuccessful");
      Serial.println("Check IMU wiring or try cycling power");
      Serial.print("Status: ");
      Serial.println(status);
      delay(1000);
    } else {
      Serial.println("init'ed mpu9250");
      break;
    }
  }

  imu.setMagCalX(CalData[0], CalData[1]);
  imu.setMagCalY(CalData[2], CalData[3]);
  imu.setMagCalZ(CalData[4], CalData[5]);

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

void getCalibratedData() {
  std::stringstream ss;
  ss.precision(1);

  ss << std::fixed << "bX=" << imu.getMagBiasX_uT() << ", "
     << "bY=" << imu.getMagBiasY_uT() << ", "
     << "bZ=" << imu.getMagBiasZ_uT() << ", "
     << "sX=" << imu.getMagScaleFactorX() << ", "
     << "sY=" << imu.getMagScaleFactorY() << ", "
     << "sZ=" << imu.getMagScaleFactorZ() << ", " << std::endl;
  Serial.println(ss.str().c_str());
}

void loop() {
  imu.readSensor();
  float magX = imu.getMagX_uT();
  float magY = imu.getMagY_uT();
  float magZ = imu.getMagZ_uT();

  std::stringstream ss;
  ss.precision(1);
  ss << std::fixed << "Mag (uT) [" << magX << ", " << magY << ", " << magZ
     << "]" << std::endl;
  Serial.println(ss.str().c_str());

  auto direction = get_direction(magX, magY, magZ);
  // auto direction = get_direction(-11.3, 14.3, -47.4);
  Serial.println(direction);

  auto heading = get_heading(magX, magY, magZ);

  delay(3000);
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

String get_heading(float hx, float hy, float hz) {
  float h = sqrtf(hx * hx + hy * hy + hz * hz);
  hx /= h;
  hy /= h;
  hz /= h;

  /* Compute euler angles */
  float yaw_rad = atan2f(-hy, hx);
  float heading_rad = constrainAngle360(yaw_rad);
  /* Filtering heading */
  float filtered_heading_rad =
      (filtered_heading_rad * (window_size - 1.0f) + heading_rad) / window_size;

  /* Display the results */
  Serial.print(yaw_rad * RAD_TO_DEG);
  Serial.print("\t");
  Serial.print(heading_rad * RAD_TO_DEG);
  Serial.print("\t");
  Serial.println(filtered_heading_rad * RAD_TO_DEG);

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
    direction = "SOTUH-WEST";
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
