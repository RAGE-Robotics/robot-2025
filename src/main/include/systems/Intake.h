#pragma once

#include "System.h"

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>

using namespace ctre::phoenix;

class Intake : public System {
public:
  static Intake &GetInstance() {
    static Intake instance;
    return instance;
  }

  enum Position { kIn, kOut };

  enum CoralState { kIdle, kIntaking, kIntakingSlow, kHold, kPlacing };

  enum AlgaeState { kIdleIn, kIdleOut, kGrabbing, kThrowing };

  void SetCoralState(CoralState state);
  void SetAlgaeState(AlgaeState state);
  void Update(Robot::Mode mode, double t);

private:
  Intake();

  Position m_coralPosition = kIn;
  Position m_algaePosition = kIn;

  CoralState m_coralState = kIdle;
  AlgaeState m_algaeState = kIdleIn;

  frc::DoubleSolenoid m_coralSolenoid;
  frc::DigitalInput m_firstSensor, m_secondSensor;
  motorcontrol::can::TalonSRX m_coralMotor;

  frc::DoubleSolenoid m_algaeSolenoid;
  motorcontrol::can::TalonSRX m_algaeMotor;
};
