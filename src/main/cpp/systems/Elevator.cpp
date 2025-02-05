#include "systems/Elevator.h"

#include <Constants.h>

using namespace ctre::phoenix6;

Elevator::Elevator() {
  auto [kP, kI, kD, kCV, kV, kA, kS, kAccel, kJ] =
      Constants::kElevatorMotorGains;

  ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs;
  auto &motorConfig = talonFXConfigs.Slot0;
  motorConfig.kP = kP;
  motorConfig.kI = kI;
  motorConfig.kD = kD;
  motorConfig.kV = kV;
  motorConfig.kA = kA;
  motorConfig.kS = kS;

  auto &motionMagicConfigs = talonFXConfigs.MotionMagic;
  motionMagicConfigs
      .WithMotionMagicCruiseVelocity(
          units::angular_velocity::turns_per_second_t{kCV})
      .WithMotionMagicAcceleration(
          units::angular_acceleration::turns_per_second_squared_t{kAccel})
      .WithMotionMagicJerk(units::angular_jerk::turns_per_second_cubed_t{kJ});
  m_mainMotor.GetConfigurator().Apply(talonFXConfigs);
  m_secondMotor.SetControl(controls::Follower{m_mainMotor.GetDeviceID(), true});
}

void Elevator::Update(Robot::Mode mode, double t) {
  if (mode == Robot::Mode::kAuto || mode == Robot::Mode::kTeleop) {
    controls::MotionMagicVoltage m_mainOutput{0_tr};
    switch (m_position) {
    case kL1:
      m_mainMotor.SetControl(m_mainOutput.WithPosition(
          units::turn_t{Constants::kElevatorL1PositionRotations}));
      break;
    case kL2:
      m_mainMotor.SetControl(m_mainOutput.WithPosition(
          units::turn_t{Constants::kElevatorL2PositionRotations}));
      break;
    case kL3:
      m_mainMotor.SetControl(m_mainOutput.WithPosition(
          units::turn_t{Constants::kElevatorL3PositionRotations}));
      break;
    case kL4:
      m_mainMotor.SetControl(m_mainOutput.WithPosition(
          units::turn_t{Constants::kElevatorL4PositionRotations}));
      break;
    default:
      m_mainMotor.SetControl(m_mainOutput.WithPosition(
          units::turn_t{Constants::kElevatorHomePositionRotations}));
    }
  }
}

void Elevator::SetPosition(Elevator::Position position) {
  m_position = position;
};
