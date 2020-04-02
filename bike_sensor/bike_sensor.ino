#include <ICM_20948.h>
#include <samd.h>
#include <Wire.h>

#define LED_BUILTIN 38

void setup() {
  SerialUSB.begin(115200);
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
