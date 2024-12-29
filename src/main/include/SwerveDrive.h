#pragma once

#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
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

private:
  hardware::Pigeon2 m_gyro;
  hardware::TalonFX m_steeringMotors[4];
  hardware::TalonFX m_driveMotors[4];

  // The kinematics class is what we use to convert between global velocities
  // and wheel velocities.
  frc::SwerveDriveKinematics<4> m_kinematics;

  // Make the constructor private so that the GetInstance() function must be
  // used.
  SwerveDrive();
};
