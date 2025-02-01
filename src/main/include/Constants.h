#pragma once

#include <units/length.h>

class Constants {
public:
  // Driverstation constants
  static constexpr int kDriverControllerId = 0;
  static constexpr int kOperatorControllerId = 1;

  // Constants for the drivetrain
  static constexpr int kPigeonCanId = 10;

  static constexpr auto kWheelBaseWidth = 0.5_m;

  static constexpr int kFlSteeringMotorId = 15;
  static constexpr int kFrSteeringMotorId = 16;
  static constexpr int kBlSteeringMotorId = 17;
  static constexpr int kBrSteeringMotorId = 18;
  static constexpr int kFlDriveMotorId = 11;
  static constexpr int kFrDriveMotorId = 12;
  static constexpr int kBlDriveMotorId = 13;
  static constexpr int kBrDriveMotorId = 14;

  static constexpr int kFlEncoderId = 15;
  static constexpr int kFrEncoderId = 16;
  static constexpr int kBlEncoderId = 17;
  static constexpr int kBrEncoderId = 18;

  // kS, kV, kP, kI, kD
  static constexpr std::tuple<double, double, double, double, double>
      kSteeringMotorGains{0.1, 0.1, 0.05, 0.0, 0.0};

  static constexpr double kWheelRadius = 0.7 / 2; // meters

  static constexpr double kDriveCurrentLimit = 60; // Amps
  static constexpr double kDriveRampRate = 0.25;   // Seconds
  static constexpr double kDriveVelocityMultiplier = 0.25;
  static constexpr double kDriveGearRatio = 22.0 / 72.0;

  static constexpr double kDefaultMaxV = 1.0;              // meters per second
  static constexpr double kDefaultMaxW = 1.0;              // radians per second
  static constexpr double kDefaultPositionTolerance = 0.1; // meters
  static constexpr double kDefaultAngleTolerance = 0.1;    // radians
};
