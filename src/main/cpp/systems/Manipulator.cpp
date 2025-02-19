#include "systems/Manipulator.h"

#include "Constants.h"
#include "Robot.h"

using namespace ctre::phoenix;

void Manipulator::SetCoralState(CoralState state) { m_coralState = state; }

void Manipulator::SetAlgaeState(AlgaeState state) { m_algaeState = state; }

void Manipulator::Update(Robot::Mode mode, double t) {
  if (mode == Robot::kAuto || mode == Robot::kTeleop) {
    if (m_coralState == kIntaking) {
      m_coralSolenoid.Set(frc::DoubleSolenoid::kReverse);
      // Manipulator motor fast
      m_coralMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                       Constants::kManipulatorCoralSpeedFast);

      if (m_secondSensor.Get()) {
        m_coralState = kIntakingSlow;
      }
    } else if (m_coralState == kIntakingSlow) {
      m_coralSolenoid.Set(frc::DoubleSolenoid::kReverse);
      // Move Manipulator wheels slowly
      m_coralMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                       Constants::kManipulatorCoralSpeedSlow);

      if (!m_firstSensor.Get()) {
        m_coralState = kHold;
      }
    } else if (m_coralState == kHold) {
      m_coralSolenoid.Set(frc::DoubleSolenoid::kReverse);

      if (m_firstSensor.Get()) {
        // Turn off motors
        m_coralMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput, 0);

      } else {
        // Run motors in reverse slowly
        m_coralMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                         Constants::kManipulatorCoralSpeedReverse);
      }
    } else if (m_coralState == kPlacing) {
      m_coralSolenoid.Set(frc::DoubleSolenoid::kForward);
      // Move Manipulator wheels fast
      m_coralMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                       Constants::kManipulatorCoralSpeedFast);

      if (!m_secondSensor.Get()) {
        m_coralState = kIdle;
      }
    }

    if (m_algaeState == kIdleIn) {
      m_algaeSolenoid.Set(frc::DoubleSolenoid::kReverse);
      m_algaeMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput, 0);
    } else if (m_algaeState == kIdleOut) {
      m_algaeSolenoid.Set(frc::DoubleSolenoid::kForward);
      m_algaeMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput, 0);
    } else if (m_algaeState == kGrabbing) {
      m_algaeSolenoid.Set(frc::DoubleSolenoid::kForward);
      m_algaeMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                       Constants::kManipulatorAlgaeManipulatorSpeed);
    } else if (m_algaeState == kThrowing) {
      m_algaeSolenoid.Set(frc::DoubleSolenoid::kForward);
      m_algaeMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                       Constants::kManipulatorAlgaeOuttakeSpeed);
    }
  }
}

Manipulator::Manipulator()
    : m_coralSolenoid{frc::PneumaticsModuleType::REVPH,
                      Constants::kManipulatorCoralSolenoidIdForward,
                      Constants::kManipulatorCoralSolenoidIdReverse},
      m_algaeSolenoid{frc::PneumaticsModuleType::REVPH,
                      Constants::kManipulatorAlgaeSolenoidIdForward,
                      Constants::kManipulatorAlgaeSolenoidIdReverse},
      m_firstSensor{Constants::kManipulatorFirstSensorId},
      m_secondSensor{Constants::kManipulatorSecondSensorId},
      m_coralMotor{Constants::kManipulatorCoralMotorId},
      m_algaeMotor{Constants::kManipulatorAlgaeMotorId} {}
