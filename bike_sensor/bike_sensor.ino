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

// 8 fig vertical and hoz: bX=39.0, bY=52.0, bZ=18.1, sX=1.0, sY=0.8, sZ=1.4,
// Scale X, bias X, scale Y, bias Y...
std::vector<float> MpuCalibrated = {1.0, 39.0, 0.8, 52.0, 1.4, 1.0};

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

  delay(2500);
  Serial.print(
      "Calibrating magnetometer, please slowly move in a figure 8 until "
      "complete...");
  imu.calibrateMag();
  Serial.println("Done!");
  Serial.print("Saving results to EEPROM...");
  /* Save to EEPROM */
  value = imu.getMagBiasX_uT();
  memcpy(eeprom_buffer, &value, sizeof(value));
  value = imu.getMagBiasY_uT();
  memcpy(eeprom_buffer + 4, &value, sizeof(value));
  value = imu.getMagBiasZ_uT();
  memcpy(eeprom_buffer + 8, &value, sizeof(value));
  value = imu.getMagScaleFactorX();
  memcpy(eeprom_buffer + 12, &value, sizeof(value));
  value = imu.getMagScaleFactorY();
  memcpy(eeprom_buffer + 16, &value, sizeof(value));
  value = imu.getMagScaleFactorZ();
  memcpy(eeprom_buffer + 20, &value, sizeof(value));
  for (unsigned int i = 0; i < sizeof(eeprom_buffer); i++) {
    EEPROM.write(i, eeprom_buffer[i]);
  }
  Serial.println("Done! You may power off your board.");
  getCalibratedData();
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
  // IMU.readSensor();
  getCalibratedData();
  // // // printScaledAGMT(myICM.agmt);
  // float magX = imu.getMagX_uT();
  // float magY = imu.getMagY_uT();
  // float magZ = imu.getMagZ_uT();

  // Serial.print(" ], Mag (uT) [ ");
  // printFormattedFloat(magX, 5, 2);
  // Serial.print(", ");
  // printFormattedFloat(magY, 5, 2);
  // Serial.print(", ");
  // printFormattedFloat(magZ, 5, 2);
  // Serial.print(" ]");
  // Serial.println();

  // auto direction = get_direction(magX, magY, magZ);
  // auto direction = get_direction(-11.3, 14.3, -47.4);
  // Serial.println(direction);
  // delay(30);

  delay(2500);
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

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals) {
  float aval = abs(val);
  if (val < 0) {
    Serial.print("-");
  } else {
    Serial.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++) {
    uint32_t tenpow = 0;
    if (indi < (leading - 1)) {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++) {
      tenpow *= 10;
    }
    if (aval < tenpow) {
      Serial.print("0");
    } else {
      break;
    }
  }
  if (val < 0) {
    Serial.print(-val, decimals);
  } else {
    Serial.print(val, decimals);
  }
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
