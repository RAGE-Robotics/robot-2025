#pragma once

#include <Constants.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DigitalInput.h>

#include "Robot.h"

using namespace ctre::phoenix6;

class Elevator {
public:
  enum Position { kHome, kL1, kL2, kL3, kL4 };

  static Elevator &GetInstance() {
    static Elevator instance;
    return instance;
  }

  void SetPosition(Position position);
  void Update(Robot::Mode mode, double t);
  double GetPosition();
  double GetError();

private:
  hardware::TalonFX m_mainMotor{Constants::kElevatorMainMotorId};
  hardware::TalonFX m_secondMotor{Constants::kElevatorSecondMotorId};

  Position m_position;

  Elevator();
};
