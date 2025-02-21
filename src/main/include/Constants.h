#pragma once

#include <frc/geometry/Transform3d.h>
#include <units/length.h>

class Constants {
public:
  static constexpr double kInchesPerMeter = 39.37;

  // Driverstation constants
  static constexpr int kDriverControllerId = 0;
  static constexpr int kOperatorControllerId = 1;

  // Constants for the drivetrain
  static constexpr int kPigeonCanId = 25;

  static constexpr auto kWheelBaseWidth = 24.375_in;

  static constexpr int kFlSteeringMotorId = 14;
  static constexpr int kFrSteeringMotorId = 12;
  static constexpr int kBlSteeringMotorId = 11;
  static constexpr int kBrSteeringMotorId = 17;
  static constexpr int kFlDriveMotorId = 15;
  static constexpr int kFrDriveMotorId = 16;
  static constexpr int kBlDriveMotorId = 19;
  static constexpr int kBrDriveMotorId = 13;

  static constexpr int kFlEncoderId = 24;
  static constexpr int kFrEncoderId = 23;
  static constexpr int kBlEncoderId = 22;
  static constexpr int kBrEncoderId = 21;

  // kS, kV, kP, kI, kD
  static constexpr std::tuple<double, double, double, double, double>
      kSteeringMotorGains{0.0, 0.0, 30.0, 0.0, 0.0};

  static constexpr double kWheelRadius = 0.7 / 2; // meters

  static constexpr double kDriveCurrentLimit = 60; // Amps
  static constexpr double kDriveRampRate = 0.25;   // Seconds
  static constexpr double kDriveVelocityMultiplier = 0.25;
  static constexpr double kDriveGearRatio = 22.0 / 72.0;
  static constexpr double kDriveControlMultipler = 1.0; // meters per second
  static constexpr double kDriveAngularControlMultiplier =
      1.0; // radians per second

  static constexpr double kDefaultMaxV = 1.0;              // meters per second
  static constexpr double kDefaultMaxW = 1.0;              // radians per second
  static constexpr double kDefaultPositionTolerance = 0.1; // meters
  static constexpr double kDefaultAngleTolerance = 0.1;    // radians
  static constexpr double kPathFollowingKp = 0.1;
  static constexpr double kPathFollowingKi = 0;
  static constexpr double kPathFollowingKd = 0;
  static constexpr double kPathFollowingAngleKp = 0.1;
  static constexpr double kPathFollowingAngleKi = 0;
  static constexpr double kPathFollowingAngleKd = 0;

  // Vision
  static constexpr frc::Transform3d kFrontCameraTransform{
      frc::Translation3d{0_m, 0_m, 0_m}, frc::Rotation3d{0_rad, 0_rad, 0_rad}};
  static constexpr frc::Transform3d kBackCameraTransform{
      frc::Translation3d{0_m, 0_m, 0_m}, frc::Rotation3d{0_rad, 0_rad, 0_rad}};

  // TalonFX ids for elevator motors.
  static constexpr int kElevatorMainMotorId = 18;
  static constexpr int kElevatorSecondMotorId = 20;
  static constexpr double kElevatorMetersPerRotation = 0.1715 / kInchesPerMeter;

  // Value is in meters per second
  static constexpr double kElevatorMaxVelocity =
      0.1 / kElevatorMetersPerRotation;
  // Value is in meters per second per second
  static constexpr double kElevatorAcceleration =
      0.1 / kElevatorMetersPerRotation;
  // Value in meters per second per second per second
  static constexpr double kElevatorJerk = 0.1 / kElevatorMetersPerRotation;
  // P I D CruiseVelocity Velocity S(overcome static friction) A(output per unit
  // of target acceleration)       Accel Jerk
  static constexpr std::tuple<double, double, double, double, double, double,
                              double, double, double>
      kElevatorMotorGains{0.1,
                          0,
                          0,
                          10,
                          kElevatorMaxVelocity,
                          0.01,
                          0.24,
                          kElevatorAcceleration,
                          kElevatorJerk};
  // Starting offset in meters
  static constexpr double kElevatorHomePositionMeters = 0.0;
  static constexpr double kElevatorHomePositionRotations =
      kElevatorHomePositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorL1PositionMeters = 0;
  static constexpr double kElevatorL1PositionRotations =
      kElevatorL1PositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorL2PositionMeters = 0.313;
  static constexpr double kElevatorL2PositionRotations =
      kElevatorL1PositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorL3PositionMeters = 0.719;
  static constexpr double kElevatorL3PositionRotations =
      kElevatorL1PositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorL4PositionMeters = 1.3356;
  static constexpr double kElevatorL4PositionRotations =
      kElevatorL1PositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorAlgae1PositionMeters = 0.484;
  static constexpr double kElevatorAlgae1PositionRotations =
      kElevatorAlgae1PositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorAlgae2PositionMeters = 0.89;
  static constexpr double kElevatorAlgae2PositionRotations =
      kElevatorAlgae1PositionMeters / kElevatorMetersPerRotation;

  static constexpr double kElevatorDefaultTolerance = 0.01; // meters

  // Feeder
  static constexpr int kFeederSolenoidIdForward = 0;
  static constexpr int kFeederSolenoidIdReverse = 1;

  // Manipulator
  static constexpr int kManipulatorCoralSolenoidIdForward = 2;
  static constexpr int kManipulatorCoralSolenoidIdReverse = 3;
  static constexpr int kManipulatorAlgaeSolenoidIdForward = 4;
  static constexpr int kManipulatorAlgaeSolenoidIdReverse = 5;
  static constexpr int kManipulatorFirstSensorId = 0;
  static constexpr int kManipulatorSecondSensorId = 1;
  static constexpr int kManipulatorCoralMotorId = 9;
  static constexpr int kManipulatorAlgaeMotorId = 10;
  static constexpr double kManipulatorCoralSpeedFast = 1.0;
  static constexpr double kManipulatorCoralSpeedSlow = 0.1;
  static constexpr double kManipulatorCoralSpeedReverse = -0.1;
  static constexpr double kManipulatorAlgaeManipulatorSpeed = 1.0;
  static constexpr double kManipulatorAlgaeOuttakeSpeed = -1.0;

  // Locations
  static constexpr double kReefWidth = 65.2 / kInchesPerMeter;
  static constexpr double kReefOffset =
      (144 / kInchesPerMeter) + kReefWidth / 2;
  static constexpr double kFieldLength = 649 / kInchesPerMeter;
  static constexpr double kReefSpacing = 0.5; // meters
  static constexpr double kCoralSpacing = 13 / kInchesPerMeter;
  static constexpr double kStartLineOffset = 300 / kInchesPerMeter;
  static constexpr double kStartOffsetY = 1;   // meters
  static constexpr double kStartOffsetX = 0.1; // meters
};
