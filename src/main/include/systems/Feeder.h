#pragma once

#include "System.h"

#include <frc/Solenoid.h>

class Feeder : public System {
public:
  static Feeder &GetInstance() {
    static Feeder instance;
    return instance;
  }

  enum Position { kUp, kDown };

  void SetPosition(Position position);
  void Update(Robot::Mode mode, double t);

private:
  Feeder();

  Position m_position = kDown;
  frc::Solenoid m_solenoid;
};
