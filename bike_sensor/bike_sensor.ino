
#include <Arduino.h>
// #include <Esp.h>
#include <SparkFunMPU9250-DMP.h>
#include <Wire.h>

#include "EEPROM.h"

// #include <sstream>
// #include <string>

#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_INT_PIN 19

MPU9250_DMP imu;

/* EEPROM buffer to mag bias and scale factors */
uint8_t eeprom_buffer[24];
float value;

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);

  while (imu.begin() != INV_SUCCESS) {
    Serial.println("Unable to communicate with MPU-9250");
    Serial.println("Check connections, and try again.");
    Serial.println();
    delay(5000);
  }
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  // Use setGyroFSR() and setAccelFSR() to configure the
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(2000);  // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(2);  // Set accel to +/-2g
  // Note: the MPU-9250's magnetometer FSR is set at
  // +/- 4912 uT (micro-tesla's)

  // setLPF() can be used to set the digital low-pass filter
  // of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu.setLPF(5);  // Set LPF corner frequency to 5Hz

  // The sample rate of the accel/gyro can be set using
  // setSampleRate. Acceptable values range from 4Hz to 1kHz
  imu.setSampleRate(10);  // Set sample rate to 10Hz

  // Likewise, the compass (magnetometer) sample rate can be
  // set using the setCompassSampleRate() function.
  // This value can range between: 1-100Hz
  imu.setCompassSampleRate(10);  // Set mag rate to 10Hz

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
  // IMU.readSensor();
  if (imu.dataReady()) {
    // Call update() to update the imu objects sensor data.
    // You can specify which sensors to update by combining
    // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
    // UPDATE_TEMPERATURE.
    // (The update function defaults to accel, gyro, compass,
    //  so you don't have to specify these values.)
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    printIMUData();
  }

  // //   // printScaledAGMT(myICM.agmt);
  // float magX = IMU.getMagX_uT();
  // float magY = IMU.getMagY_uT();
  // float magZ = IMU.getMagZ_uT();

  // Serial.print(" ], Mag (uT) [ ");
  // printFormattedFloat(magX, 5, 2);
  // Serial.print(", ");
  // printFormattedFloat(magY, 5, 2);
  // Serial.print(", ");
  // printFormattedFloat(magZ, 5, 2);
  // Serial.print(" ]");
  // Serial.println();

  // auto direction = get_direction(magX, magY, magZ);
  // Serial.println(direction);
  // delay(30);

  delay(2500);
}

void printIMUData(void) {
  // After calling update() the ax, ay, az, gx, gy, gz, mx,
  // my, mz, time, and/or temerature class variables are all
  // updated. Access them by placing the object. in front:

  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);
  float gyroX = imu.calcGyro(imu.gx);
  float gyroY = imu.calcGyro(imu.gy);
  float gyroZ = imu.calcGyro(imu.gz);
  float magX = imu.calcMag(imu.mx);
  float magY = imu.calcMag(imu.my);
  float magZ = imu.calcMag(imu.mz);

  Serial.println("Accel: " + String(accelX) + ", " + String(accelY) + ", " +
                 String(accelZ) + " g");
  Serial.println("Gyro: " + String(gyroX) + ", " + String(gyroY) + ", " +
                 String(gyroZ) + " dps");
  Serial.println("Mag: " + String(magX) + ", " + String(magY) + ", " +
                 String(magZ) + " uT");
  // auto direction = get_direction(magX, magY, magZ);
  auto direction = get_direction(-11.3, 14.3, -47.4);
  Serial.println(direction);
  Serial.println("Time: " + String(imu.time) + " ms");
  Serial.println();
}

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
