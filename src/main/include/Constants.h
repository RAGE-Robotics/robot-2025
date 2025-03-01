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

  static constexpr units::turn_t kEncoderOffsets[] = {
      -0.14599609375_tr, 0.302734375_tr, 0.040283203125_tr, -0.080810546875_tr};

  // kS, kV, kP, kI, kD
  static constexpr std::tuple<double, double, double, double, double>
      kSteeringMotorGains{0.0, 0.0, 30.0, 0.0, 0.0};

  static constexpr double kWheelRadius = (4.0 / kInchesPerMeter) / 2; // meters

  static constexpr double kDriveCurrentLimit = 60; // Amps
  static constexpr double kDriveRampRate = 0.35;   // Seconds
  static constexpr double kDriveVelocityMultiplier = 1 / 4.65;
  static constexpr double kDriveGearRatio = 1.0 / 6.75;
  static constexpr double kDriveControlMultipler = 3; // meters per second
  static constexpr double kDriveAngularControlMultiplier =
      5.0;                              // radians per second
  static constexpr double kMaxV = 4.65; // meters per second

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
      frc::Translation3d{6.25_in, -12.25_in, 23_in},
      frc::Rotation3d{0_rad, 25_deg, 0_rad}};
  static constexpr frc::Transform3d kBackCameraTransform{
      frc::Translation3d{0_m, 0_m, 0_m}, frc::Rotation3d{0_rad, 0_rad, 0_rad}};

  // TalonFX ids for elevator motors.
  static constexpr int kElevatorMainMotorId = 18;
  static constexpr int kElevatorSecondMotorId = 20;
  static constexpr double kElevatorMetersPerRotation = 0.1366 / 20 * 2;

  // Values is in meters per second
  static constexpr double kElevatorCruiseVelocity =
      1.0 / kElevatorMetersPerRotation;
  // Value is in meters per second per second
  static constexpr double kElevatorAcceleration =
      1.5 / kElevatorMetersPerRotation;
  // Value in meters per second per second per second
  static constexpr double kElevatorJerk = 1.0 / kElevatorMetersPerRotation;
  // P I D CruiseVelocity kV S(overcome static friction) A(output per unit
  // of target acceleration)       Accel Jerk     kG
  static constexpr std::tuple<double, double, double, double, double, double,
                              double, double, double, double>
      kElevatorMotorGains{10,
                          0,
                          0,
                          kElevatorCruiseVelocity,
                          0.126,
                          0.01,
                          0.24,
                          kElevatorAcceleration,
                          kElevatorJerk,
                          0.1};
  // Starting offset in meters
  static constexpr double kElevatorHomePositionMeters = 0.0;
  static constexpr double kElevatorHomePositionRotations =
      kElevatorHomePositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorL1PositionMeters = 0;
  static constexpr double kElevatorL1PositionRotations =
      kElevatorL1PositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorL2PositionMeters = 0.313;
  static constexpr double kElevatorL2PositionRotations =
      kElevatorL2PositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorL3PositionMeters = 0.719;
  static constexpr double kElevatorL3PositionRotations =
      kElevatorL3PositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorL4PositionMeters = 1.3356;
  static constexpr double kElevatorL4PositionRotations =
      kElevatorL4PositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorAlgae1PositionMeters = 0.434;
  static constexpr double kElevatorAlgae1PositionRotations =
      kElevatorAlgae1PositionMeters / kElevatorMetersPerRotation;
  static constexpr double kElevatorAlgae2PositionMeters = 0.83;
  static constexpr double kElevatorAlgae2PositionRotations =
      kElevatorAlgae2PositionMeters / kElevatorMetersPerRotation;

  static constexpr double kElevatorDefaultTolerance = 0.01;      // meters
  static constexpr double kElevatorCollisionTripDistance = 0.05; // meters

  // Manipulator
  static constexpr int kManipulatorCoralSolenoidIdForward = 14;
  static constexpr int kManipulatorCoralSolenoidIdReverse = 13;
  static constexpr int kManipulatorAlgaeSolenoidIdForward = 12;
  static constexpr int kManipulatorAlgaeSolenoidIdReverse = 11;
  static constexpr int kManipulatorFirstSensorId = 0;
  static constexpr int kManipulatorSecondSensorId = 1;
  static constexpr int kManipulatorCoralMotorId = 10;
  static constexpr int kManipulatorAlgaeMotorId = 6;
  static constexpr double kManipulatorCoralIntakeSpeed = 0.25;
  static constexpr double kManipulatorCoralOutakeSpeed = 1.0;
  static constexpr double kManipulatorCoralSpeedReverse = -0.15;
  static constexpr double kManipulatorAlgaeManipulatorSpeed = 1.0;
  static constexpr double kManipulatorAlgaeOutakeSpeed = -1.0;
  static constexpr double kManipulatorAlgaeHoldSpeed = 0.4;

  // Locations
  static constexpr double kReefWidth = 65.2 / kInchesPerMeter;
  static constexpr double kReefOffset =
      (144 / kInchesPerMeter) + kReefWidth / 2;
  static constexpr double kFieldLength = 690.875 / kInchesPerMeter;
  static constexpr double kFieldWidth = 317 / kInchesPerMeter;
  static constexpr double kReefSpacing = 0.5; // meters
  static constexpr double kCoralSpacing = 13 / kInchesPerMeter;
  static constexpr double kStartLineOffset = 300 / kInchesPerMeter;
  static constexpr double kStartOffsetY = 1;   // meters
  static constexpr double kStartOffsetX = 0.1; // meters

  // Pneumatics
  static constexpr double kMinPressure = 100; // PSI
  static constexpr double kMaxPressure = 120; // PSI

  // Status feed
  static constexpr int kStatusFrameWidth = 640;
  static constexpr int kStatusFrameHeight = 480;
  static constexpr double kStatusFrameScale = 80;
};
