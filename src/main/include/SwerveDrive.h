#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include "Robot.h"

using namespace ctre::phoenix6;

class SwerveDrive {
public:
  static SwerveDrive &GetInstance() {
    static SwerveDrive instance;
    return instance;
  }

  void Update(Robot::Mode mode);
  frc::Rotation2d GetGyroRotation2d() const;
  frc::Pose2d GetPose2d() const;
  void Coast();
  void Brake();

private:
  // The gyroscope keeps track of which direction the robot is facing.
  hardware::Pigeon2 m_gyro;

  // The steering motors turn the direction of the wheels on the swerve modules.
  hardware::TalonFX m_steeringMotors[4];
  hardware::TalonFX m_driveMotors[4];

  // These encoders keep track of the direction the swerve modules are pointing.
  hardware::CANcoder m_encoders[4];

  // The kinematics class is what we use to convert between global velocities
  // and wheel velocities.
  frc::SwerveDriveKinematics<4> m_kinematics;

  // The pose estimator keeps track of the robot's position on the field.
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

  // Make the constructor private so that the GetInstance() function must be
  // used.
  SwerveDrive();
};
