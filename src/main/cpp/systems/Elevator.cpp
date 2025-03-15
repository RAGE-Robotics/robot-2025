#include "systems/Elevator.h"
#include "systems/Manipulator.h"

#include <Constants.h>

using namespace ctre::phoenix6;

void Elevator::SetPosition(Elevator::Position position) {
  if (Manipulator::GetInstance().ElevatorSafe()) {
    m_position = position;
  }
};

Elevator::Elevator() : m_homeSwitch{Constants::kElevatorHomeSensorId} {
  auto [kP, kI, kD, kCV, kV, kA, kS, kAccel, kJ, kG] =
      Constants::kElevatorMotorGains;

  ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs;
  auto &motorConfig = talonFXConfigs.Slot0;
  motorConfig.kP = kP;
  motorConfig.kI = kI;
  motorConfig.kD = kD;
  motorConfig.kV = kV;
  motorConfig.kA = kA;
  motorConfig.kS = kS;
  motorConfig.kG = kG;

  auto &motionMagicConfigs = talonFXConfigs.MotionMagic;
  motionMagicConfigs
      .WithMotionMagicCruiseVelocity(
          units::angular_velocity::turns_per_second_t{kCV})
      .WithMotionMagicAcceleration(
          units::angular_acceleration::turns_per_second_squared_t{kAccel})
      .WithMotionMagicJerk(units::angular_jerk::turns_per_second_cubed_t{kJ});
  m_mainMotor.GetConfigurator().Apply(talonFXConfigs);
  m_secondMotor.SetControl(controls::Follower{m_mainMotor.GetDeviceID(), true});

  auto currentLimitConfig = configs::CurrentLimitsConfigs{}
                                .WithSupplyCurrentLimitEnable(true)
                                .WithSupplyCurrentLimit(units::ampere_t{
                                    Constants::kElevatorCurrentLimit});
  m_mainMotor.GetConfigurator().Apply(currentLimitConfig);
  m_secondMotor.GetConfigurator().Apply(currentLimitConfig);
}

void Elevator::Update(Robot::Mode mode, double t) {
  if (mode == Robot::Mode::kAuto || mode == Robot::Mode::kTeleop) {
    m_mainMotor.SetNeutralMode(signals::NeutralModeValue::Coast);
    m_secondMotor.SetNeutralMode(signals::NeutralModeValue::Coast);

    controls::MotionMagicVoltage m_mainOutput{0_tr};
    switch (m_position) {
    case kHome:
      m_mainMotor.SetControl(m_mainOutput
                                 .WithPosition(units::turn_t{
                                     Constants::kElevatorHomePositionRotations})
                                 .WithSlot(0));
      break;
    case kL1:
      m_mainMotor.SetControl(m_mainOutput
                                 .WithPosition(units::turn_t{
                                     Constants::kElevatorL1PositionRotations})
                                 .WithSlot(0));
      break;
    case kL2:
      m_mainMotor.SetControl(m_mainOutput
                                 .WithPosition(units::turn_t{
                                     Constants::kElevatorL2PositionRotations})
                                 .WithSlot(0));
      break;
    case kL3:
      m_mainMotor.SetControl(m_mainOutput
                                 .WithPosition(units::turn_t{
                                     Constants::kElevatorL3PositionRotations})
                                 .WithSlot(0));
      break;
    case kL4:
      m_mainMotor.SetControl(m_mainOutput
                                 .WithPosition(units::turn_t{
                                     Constants::kElevatorL4PositionRotations})
                                 .WithSlot(0));
      break;
    case kAlgae1:
      m_mainMotor.SetControl(
          m_mainOutput
              .WithPosition(
                  units::turn_t{Constants::kElevatorAlgae1PositionRotations})
              .WithSlot(0));
      break;
    case kAlgae2:
      m_mainMotor.SetControl(
          m_mainOutput
              .WithPosition(
                  units::turn_t{Constants::kElevatorAlgae2PositionRotations})
              .WithSlot(0));
      break;
    default:
      m_mainMotor.SetControl(m_mainOutput
                                 .WithPosition(units::turn_t{
                                     Constants::kElevatorHomePositionRotations})
                                 .WithSlot(0));
    }
  } else {
    m_mainMotor.SetNeutralMode(signals::NeutralModeValue::Brake);
    m_secondMotor.SetNeutralMode(signals::NeutralModeValue::Brake);
  }
}

double Elevator::GetPosition() {
  return m_mainMotor.GetPosition().GetValueAsDouble() *
         Constants::kElevatorMetersPerRotation;
}

double Elevator::GetError() {
  double setpoint;
  switch (m_position) {
  case kL1:
    setpoint = Constants::kElevatorL1PositionMeters;
    break;
  case kL2:
    setpoint = Constants::kElevatorL2PositionMeters;
    break;
  case kL3:
    setpoint = Constants::kElevatorL3PositionMeters;
    break;
  case kAlgae1:
    setpoint = Constants::kElevatorL4PositionMeters;
    break;
  case kAlgae2:
    setpoint = Constants::kElevatorL4PositionMeters;
    break;
  default:
    setpoint = Constants::kElevatorHomePositionMeters;
  }

  return std::abs(GetPosition() - setpoint);
}
