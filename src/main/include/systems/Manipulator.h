#pragma once

#include "System.h"

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <rev/SparkMax.h>
#include <Constants.h>

using namespace ctre::phoenix;

class Manipulator : public System {
public:
  static Manipulator &GetInstance() {
    static Manipulator instance;
    return instance;
  }

  void ExtendArm();
  void RetractArm();
  void SetAlgaeSpeed(double speed);
  void StartIntakingCoral();
  void StopIntakingCoral();
  void StartOutakingCoral();
  void StopOutakingCoral();
  bool ArmDown();
  bool ElevatorSafe();
  bool DoneIntaking();
  void Update(Robot::Mode mode, double t);

private:
  enum Position { kIn, kOut };

  Manipulator();

  bool m_armOut = false;
  double m_algaeSpeed = 0;
  bool m_coralIntaking = false;
  bool m_coralOutaking = false;

  frc::DoubleSolenoid m_coralSolenoid;
  frc::DigitalInput m_firstSensor, m_secondSensor;
  motorcontrol::can::TalonSRX m_coralMotor;
  frc::DigitalInput m_elevatorBlockSensor;
  frc::DoubleSolenoid m_algaeSolenoid;
  motorcontrol::can::TalonSRX m_algaeMotor;
  rev::spark::SparkMax m_assist{Constants::kManipulatorAssistID, rev::spark::SparkMax::MotorType::kBrushless};

};
