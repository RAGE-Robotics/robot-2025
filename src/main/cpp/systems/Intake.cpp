#include "Systems/Intake.h"

#include "Constants.h"
#include "Robot.h"

using namespace ctre::phoenix;

void Intake::SetCoralState(CoralState state) { m_coralState = state; }

void Intake::SetAlgaeState(AlgaeState state) { m_algaeState = state; }

void Intake::Update(Robot::Mode mode, double t) {
  if (mode == Robot::kAuto || mode == Robot::kTeleop) {
    if (m_coralState == kIntaking) {
      m_coralSolenoid.Set(frc::DoubleSolenoid::kReverse);
      // Intake motor fast
      m_coralMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                       Constants::kIntakeCoralSpeedFast);

      if (m_secondSensor.Get()) {
        m_coralState = kIntakingSlow;
      }
    } else if (m_coralState == kIntakingSlow) {
      m_coralSolenoid.Set(frc::DoubleSolenoid::kReverse);
      // Move intake wheels slowly
      m_coralMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                       Constants::kIntakeCoralSpeedSlow);

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
                         Constants::kIntakeCoralSpeedReverse);
      }
    } else if (m_coralState == kPlacing) {
      m_coralSolenoid.Set(frc::DoubleSolenoid::kForward);
      // Move intake wheels fast
      m_coralMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                       Constants::kIntakeCoralSpeedFast);

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
                       Constants::kIntakeAlgaeIntakeSpeed);
    } else if (m_algaeState == kThrowing) {
      m_algaeSolenoid.Set(frc::DoubleSolenoid::kForward);
      m_algaeMotor.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                       Constants::kIntakeAlgaeOuttakeSpeed);
    }
  }
}

Intake::Intake()
    : m_coralSolenoid{0, frc::PneumaticsModuleType::REVPH,
                      Constants::kIntakeCoralSolenoidIdForward,
                      Constants::kIntakeCoralSolenoidIdReverse},
      m_algaeSolenoid{0, frc::PneumaticsModuleType::REVPH,
                      Constants::kIntakeAlgaeSolenoidIdForward,
                      Constants::kIntakeAlgaeSolenoidIdReverse},
      m_firstSensor{Constants::kIntakeFirstSensorId},
      m_secondSensor{Constants::kIntakeSecondSensorId},
      m_coralMotor{Constants::kIntakeCoralMotorId},
      m_algaeMotor{Constants::kIntakeAlgaeMotorId} {}
