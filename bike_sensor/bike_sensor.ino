#include <Adafruit_BME280.h>
#include <ICM_20948.h>
#include <Wire.h>
#include <samd.h>

ICM_20948_I2C myICM;
Adafruit_BME280 bme;
auto icm_init = false;

#define AD0_VAL 0
#define BME280_ADDR 0x76  // or 0x77
#define LED_BUILTIN 38

void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB)
    ;
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  SerialUSB.println("sda:" + String(PIN_WIRE_SDA));
  SerialUSB.println("scl:" + String(PIN_WIRE_SCL));
  Wire.begin();

  if (!bme.begin(BME280_ADDR, &Wire)) {
    Serial.println(
        "Could not find a valid BME280 sensor, check wiring, address, sensor "
        "ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
    Serial.print(
        "        ID of 0xFF probably means a bad address, a BMP 180 or BMP "
        "085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
  }
  // Wire.setClock(400000);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  // Wire.beginTransmission(0x55);
  // Wire.write(0);
  // Wire.endTransmission(0x55);
  auto temp = bme.readTemperature();
  SerialUSB.println("temp:" + String(temp));
  delay(800);
  // if (!icm_init) {
  //   SerialUSB.println("begining icm");
  //   myICM.begin(Wire, AD0_VAL);
  //   SerialUSB.println("done icm");
  //   SerialUSB.println(myICM.statusString());
  //   if (myICM.status != ICM_20948_Stat_Ok) {
  //     SerialUSB.println("Trying again...");
  //     // delay(500);
  //   } else {
  //     icm_init = true;
  //   }
  // }
  digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
  delay(800);
}
