#include <Adafruit_BME280.h>
#include <MPU9250.h>
#include <Wire.h>
#include <quaternionFilters.h>
#include <samd.h>

#define I2C_CLOCK 400000
// #define MPU9250_ADDRESS MPU9250_ADDRESS_AD1
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0
#define BME280_ADDR 0x76  // or 0x77
#define LED_BUILTIN 38

MPU9250 myIMU(MPU9250_ADDRESS, Wire, I2C_CLOCK);

void InitIMU(Serial_ Serial, MPU9250 myIMU) {
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  if (c == 0x71)  // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0], 1);
    Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1], 1);
    Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2], 1);
    Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3], 1);
    Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4], 1);
    Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5], 1);
    Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

    if (d != 0x48) {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    //  Serial.println("Calibration values: ");
    Serial.print("X-Axis factory sensitivity adjustment value ");
    Serial.println(myIMU.factoryMagCalibration[0], 2);
    Serial.print("Y-Axis factory sensitivity adjustment value ");
    Serial.println(myIMU.factoryMagCalibration[1], 2);
    Serial.print("Z-Axis factory sensitivity adjustment value ");
    Serial.println(myIMU.factoryMagCalibration[2], 2);

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();
  }
}

void setup() {
  SerialUSB.begin(115200);
  // while (!SerialUSB)
  //   ;
  delay(500);
  delay(5500);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Wire.setClock(400000);

  InitIMU(SerialUSB, myIMU);

  int status = -1;
}

void loop() {
  // IMU.readSensor();
  SerialUSB.println("loop");
  // byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  // SerialUSB.print(F("MPU9250 I AM 0x"));
  // SerialUSB.print(c, HEX);
  // SerialUSB.print(F(" I should be 0x"));
  // SerialUSB.println(0x71, HEX);

  //   // printScaledAGMT(myICM.agmt);
  // float magX = IMU.getMagX_uT();
  // float magY = IMU.getMagY_uT();
  // float magZ = IMU.getMagZ_uT();

  // SerialUSB.print(" ], Mag (uT) [ ");
  // printFormattedFloat(magX, 5, 2);
  // SerialUSB.print(", ");
  // printFormattedFloat(magY, 5, 2);
  // SerialUSB.print(", ");
  // printFormattedFloat(magZ, 5, 2);
  // SerialUSB.print(" ]");
  // SerialUSB.println();

  // auto direction = get_direction(magX, magY, magZ);
  // SerialUSB.println(direction);
  // delay(30);

  delay(2000);
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals) {
  float aval = abs(val);
  if (val < 0) {
    SerialUSB.print("-");
  } else {
    SerialUSB.print(" ");
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
      SerialUSB.print("0");
    } else {
      break;
    }
  }
  if (val < 0) {
    SerialUSB.print(-val, decimals);
  } else {
    SerialUSB.print(val, decimals);
  }
}

String get_direction(float x, float y, float z) {
  int heading = 0;

  if (y > 0) {
    heading = 90 - atan(x / y) * RAD_TO_DEG;
  } else if (y < 0) {
    heading = 270 - atan(x / y) * RAD_TO_DEG;
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
