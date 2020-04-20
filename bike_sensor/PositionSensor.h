
// Control the position sensor for the BikeLight
#ifndef POSITIONSENSOR_h
#define POSITIONSENSOR_h

#include <Arduino.h>
#include <MPU9250.h>
#include <Wire.h>

#include "SPI.h"
#include "bike_light_util.h"

#define MPU9250_ADDRESS 0x68

namespace BikeLight {
class PositionSensor {
 public:
  PositionSensor(TwoWire &bus) : imu(bus, MPU9250_ADDRESS) {}
  PositionSensor() : imu(Wire, MPU9250_ADDRESS) {}

  int begin() {
    int status = imu.begin();
    if (status < 0) {
      return status;
    }

    imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_10HZ);
    imu.setSrd(19);

    imu.setMagCalX(MagCal[0], MagCal[1]);
    imu.setMagCalY(MagCal[2], MagCal[3]);
    imu.setMagCalZ(MagCal[4], MagCal[5]);

    return status;
  }

  float read_heading() {
    imu.readSensor();

    float ax = imu.getAccelX_mss();
    float ay = imu.getAccelY_mss();
    float az = imu.getAccelZ_mss();

    float hx = imu.getMagX_uT();
    float hy = imu.getMagY_uT();
    float hz = imu.getMagZ_uT();

    /* Normalize accelerometer and magnetometer data */
    float a = sqrtf(ax * ax + ay * ay + az * az);
    ax /= a;
    ay /= a;
    az /= a;

    float h = sqrtf(hx * hx + hy * hy + hz * hz);
    hx /= h;
    hy /= h;
    hz /= h;

    float pitch_rad = asinf(ax);
    float roll_rad = asinf(-ay / cosf(pitch_rad));
    float yaw_rad =
        atan2f(hz * sinf(roll_rad) - hy * cosf(roll_rad),
               hx * cosf(pitch_rad) + hy * sinf(pitch_rad) * sinf(roll_rad) +
                   hz * sinf(pitch_rad) * cosf(roll_rad));
    float heading_rad = constrainAngle360(yaw_rad);

    float heading_deg = heading_rad * RAD_TO_DEG;

    return heading_deg;
  }

  std::string read_direction() {
    float heading = read_heading();
    return heading_to_direction(heading);
  }

  void print_calibrated_mag() {
    std::stringstream ss = get_fixed_stringstream();

    ss << "bX=" << imu.getMagBiasX_uT() << ", "
       << "bY=" << imu.getMagBiasY_uT() << ", "
       << "bZ=" << imu.getMagBiasZ_uT() << ", "
       << "sX=" << imu.getMagScaleFactorX() << ", "
       << "sY=" << imu.getMagScaleFactorY() << ", "
       << "sZ=" << imu.getMagScaleFactorZ() << ", " << std::endl;
    Serial.println(ss.str().c_str());
  }

 private:
  float constrainAngle360(float dta) {
    dta = fmod(dta, 2.0 * PI);
    if (dta < 0.0) dta += 2.0 * PI;
    return dta;
  }

  std::string heading_to_direction(float heading) const {
    std::string direction = "";
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
    return direction;
  }

  MPU9250 imu;
  std::vector<float> MagCal = {39.0, 1.0, 52.0, 0.8, 18.1, 1.4};
};
}  // namespace BikeLight

#endif