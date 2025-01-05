#include "Robot.h"

#include <frc/Timer.h>

#include "SwerveDrive.h"

// This gets called first. So, initialize everything here.
Robot::Robot() {
  // Call GetInstance() so the constructors get called
  SwerveDrive::GetInstance();

  // This initializes the main looper. What you put here will run @200 Hz while
  // the robot is on.
  m_looper = Looper{[this] {
    Mode mode;
    if (IsAutonomous()) {
      mode = kAutonomous;
    } else if (IsTeleop()) {
      mode = kTeleop;
    } else {
      mode = kDisabled;
    }

    SwerveDrive::GetInstance().Update(mode,
                                      frc::Timer::GetFPGATimestamp().value());
  }};
}

// This destructor gets called when the robot program shuts down.
// Cleanup any resources (especially files) before the robot code gets
// restarted.
Robot::~Robot() {}

#ifndef RUNNING_FRC_TESTS
int main(int argc, char **argv) { frc::StartRobot<Robot>(); }
#endif
