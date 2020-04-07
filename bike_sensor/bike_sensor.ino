#include <Adafruit_BME280.h>
// #include <ICM_20948.h>
#include <Wire.h>
#include <samd.h>

#include "ICM20948.h"

// ICM_20948_I2C myICM;
Adafruit_BME280 bme;

#define AD0_VAL 0
#define BME280_ADDR 0x76  // or 0x77
#define LED_BUILTIN 38

ICM20948 IMU(Wire, 0x68);

void setup() {
  SerialUSB.begin(115200);
  // while (!SerialUSB)
  //   ;
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Wire.setClock(400000);
  // SerialUSB.println("begining bme280");
  // if (!bme.begin(BME280_ADDR, &Wire)) {
  //   SerialUSB.println(
  //       "Could not find a valid BME280 sensor, check wiring, address, "
  //       "sensorID!");
  //   SerialUSB.print("SensorID was: 0x");
  //   SerialUSB.print(
  //       "        ID of 0xFF probably means a bad address, a BMP 180 or BMP "
  //       "085\n");
  //   SerialUSB.print("   ID of 0x56-0x58 represents a BMP 280,\n");
  //   SerialUSB.print("        ID of 0x60 represents a BME 280.\n");
  //   SerialUSB.print("        ID of 0x61 represents a BME 680.\n");
  // }
  // SerialUSB.println("done bme280");

  auto icm_init = false;

  // while (!icm_init) {
  //   SerialUSB.println("begining icm");
  //   myICM.begin(Wire, AD0_VAL);
  //   SerialUSB.println(myICM.statusString());
  //   if (myICM.status != ICM_20948_Stat_Ok) {
  //     SerialUSB.println("Trying again...");
  //     delay(500);
  //   } else {
  //     icm_init = true;
  //     SerialUSB.println("init!");
  //   }
  // }

  int status = -1;
  while (status < 0) {
    status = IMU.begin();
    SerialUSB.print("status = ");
    SerialUSB.println(status);
    if (status < 0) {
      SerialUSB.println("IMU initialization unsuccessful");
      SerialUSB.println("Check IMU wiring or try cycling power");
      SerialUSB.print("Status: ");
      SerialUSB.println(status);
      delay(500);
    }
  }
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
  // int heading = atan2(x, y);
  // if (heading < 0) {
  //   heading += 360;
  //   heading = 360 - heading;
  // }
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

// void printScaledAGMT(ICM_20948_AGMT_t agmt) {
//   // SerialUSB.print("Scaled. Acc (mg) [ ");
//   // printFormattedFloat(myICM.accX(), 5, 2);
//   // SerialUSB.print(", ");
//   // printFormattedFloat(myICM.accY(), 5, 2);
//   // SerialUSB.print(", ");
//   // printFormattedFloat(myICM.accZ(), 5, 2);
//   // SerialUSB.print(" ], Gyr (DPS) [ ");
//   // printFormattedFloat(myICM.gyrX(), 5, 2);
//   // SerialUSB.print(", ");
//   // printFormattedFloat(myICM.gyrY(), 5, 2);
//   // SerialUSB.print(", ");
//   // printFormattedFloat(myICM.gyrZ(), 5, 2);
//   SerialUSB.print(" ], Mag (uT) [ ");
//   printFormattedFloat(myICM.magX(), 5, 2);
//   SerialUSB.print(", ");
//   printFormattedFloat(myICM.magY(), 5, 2);
//   SerialUSB.print(", ");
//   printFormattedFloat(myICM.magZ(), 5, 2);
//   // SerialUSB.print(" ], Tmp (C) [ ");
//   // printFormattedFloat(myICM.temp(), 5, 2);
//   SerialUSB.print(" ]");
//   SerialUSB.println();
// }

void loop() {
  IMU.readSensor();

  //   // printScaledAGMT(myICM.agmt);
  float magX = IMU.getMagX_uT();
  float magY = IMU.getMagY_uT();
  float magZ = IMU.getMagZ_uT();

  SerialUSB.print(" ], Mag (uT) [ ");
  printFormattedFloat(magX, 5, 2);
  SerialUSB.print(", ");
  printFormattedFloat(magY, 5, 2);
  SerialUSB.print(", ");
  printFormattedFloat(magZ, 5, 2);
  SerialUSB.print(" ]");
  SerialUSB.println();

  auto direction = get_direction(magX, magY, magZ);
  SerialUSB.println(direction);
  delay(30);

  // auto temp = bme.readTemperature();
  // SerialUSB.println("temp:" + String(temp));
  // if (myICM.dataReady()) {
  //   myICM.getAGMT();
  //   // printScaledAGMT(myICM.agmt);
  //   // float magX = myICM.magX() * 10000;
  //   // float magY = myICM.magY() * 10000;
  //   // float magZ = myICM.magZ() * 10000;

  //   float magX = myICM.magX();
  //   float magY = myICM.magY();
  //   float magZ = myICM.magZ();

  //   // float magX = myICM.agmt.mag.axes.x;
  //   // float magY = myICM.agmt.mag.axes.y;
  //   // float magZ = myICM.agmt.mag.axes.z;

  //   SerialUSB.print(" ], Mag (uT) [ ");
  //   printFormattedFloat(magX, 5, 2);
  //   SerialUSB.print(", ");
  //   printFormattedFloat(magY, 5, 2);
  //   SerialUSB.print(", ");
  //   printFormattedFloat(magZ, 5, 2);
  //   SerialUSB.print(" ]");
  //   SerialUSB.println();

  //   auto direction = get_direction(magX, magY, magZ);
  //   SerialUSB.println(direction);
  //   delay(30);
  // } else {
  //   SerialUSB.println("Waiting for data");
  //   delay(500);
  // }
  delay(2000);

  // // SerialUSB.println("reset: " + String(icm_init));
  // digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage
  // LOW delay(800);
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
