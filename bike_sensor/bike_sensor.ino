#include <ICM_20948.h>
#include <Wire.h>
#include <samd.h>

ICM_20948_I2C myICM;

#define AD0_VAL 1
#define LED_BUILTIN 38

void setup() {
  SerialUSB.begin(115200);

  Wire.begin();
  Wire.setClock(400000);
  auto icm_init = false;
  while (!icm_init) {
    myICM.begin(Wire, AD0_VAL);
    SerialUSB.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      SerialUSB.println("Trying again...");
      delay(500);
    } else {
      icm_init = true;
    }
  }

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  SerialUSB.println("hey yo2");
  digitalWrite(LED_BUILTIN,
               HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(800);          // wait for a second
  digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
  delay(800);
}
