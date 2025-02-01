#pragma once

#include <frc/TimedRobot.h>

#include "Looper.h"

class Robot : public frc::TimedRobot {
public:
  // Enum to specify the robot's mode to the sub-systems
  enum Mode { kDisabled, kAuto, kTeleop };

  Robot();
  ~Robot();

private:
  Looper m_looper;
  Mode m_mode;
};
