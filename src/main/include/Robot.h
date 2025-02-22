#pragma once

#include <frc/PneumaticHub.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <memory>

#include "Looper.h"
#include "auto/Task.h"

class Robot : public frc::TimedRobot {
public:
  // Enum to specify the robot's mode to the sub-systems
  enum Mode { kDisabled, kAuto, kTeleop };

  Robot();
  ~Robot();

  void DisabledExit() override;

private:
  frc::PneumaticHub m_pneumaticsHub;

  Looper m_looper;

  frc::SendableChooser<int> m_startChooser;
  frc::SendableChooser<std::string> m_autoChooser;

  std::shared_ptr<Task> m_auto;
};
