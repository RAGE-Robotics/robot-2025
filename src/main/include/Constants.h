#pragma once

#include <units/length.h>

class Constants {
public:
  static constexpr int kPigeonCanId = 10;
  static constexpr auto kWheelBaseWidth = 0.5_m;
  static constexpr int kFlDriveMotorId = 11;
  static constexpr int kFrDriveMotorId = 12;
  static constexpr int kBlDriveMotorId = 13;
  static constexpr int kBrDriveMotorId = 14;
  static constexpr int kFlSteeringMotorId = 15;
  static constexpr int kFrSteeringMotorId = 16;
  static constexpr int kBlSteeringMotorId = 17;
  static constexpr int kBrSteeringMotorId = 18;

  // kS, kV, kP, kI, kD
  static constexpr std::tuple<double, double, double, double, double>
      kSteeringMotorGains{0.1, 0.1, 0.05, 0.0, 0.0};

  static constexpr double kWheelRadius = 0.7 / 2; // meters
};
