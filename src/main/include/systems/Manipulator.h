#pragma once

#include "System.h"

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>

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
  void StopIntakingcoral();
  void Update(Robot::Mode mode, double t);

private:
  enum Position { kIn, kOut };

  Manipulator();

  bool m_armOut = false;
  double m_algaeSpeed = 0;
  bool m_coralIntaking = true;

  frc::DoubleSolenoid m_coralSolenoid;
  frc::DigitalInput m_firstSensor, m_secondSensor;
  motorcontrol::can::TalonSRX m_coralMotor;

  frc::DoubleSolenoid m_algaeSolenoid;
  motorcontrol::can::TalonSRX m_algaeMotor;
};
