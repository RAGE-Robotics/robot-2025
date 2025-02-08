#include "Robot.h"

#include <frc/DriverStation.h>
#include <frc/Timer.h>

#include "Controllers.h"
#include "Locations.h"
#include "Util.h"
#include "systems/Cameras.h"
#include "systems/Elevator.h"
#include "systems/Feeder.h"
#include "systems/Manipulator.h"
#include "systems/SwerveDrive.h"

// This gets called first. So, initialize everything here.
Robot::Robot() {
  // Call GetInstance() so the constructors get called
  SwerveDrive::GetInstance();

  // This initializes the main looper. What you put here will run @200 Hz while
  // the robot is on.
  m_looper = Looper{[this] {
    Mode mode;
    if (IsAutonomous()) {
      mode = kAuto;
    } else if (IsTeleop()) {
      mode = kTeleop;
    } else {
      mode = kDisabled;
    }

    if (mode == kTeleop) {
      // Get the inputs from the controller during teleop mode. Note this uses
      // the split setup where the left joystick controls velocity, and the
      // right joystick controls the rotation. The Util::exp() function squares
      // the input while keeping the sign.
      double vx = Util::Exp(
          -Controllers::GetInstance().GetDriverController().GetLeftY());
      double vy = Util::Exp(
          -Controllers::GetInstance().GetDriverController().GetLeftX());
      double w = Util::Exp(
          -Controllers::GetInstance().GetDriverController().GetRightX());
      SwerveDrive::GetInstance().DriveVelocity(vx, vy, w);
    }

    double t = frc::Timer::GetFPGATimestamp().value();
    Cameras::GetInstance().Update(mode, t);
    SwerveDrive::GetInstance().Update(mode, t);
    Elevator::GetInstance().Update(mode, t);
    Feeder::GetInstance().Update(mode, t);
    Manipulator::GetInstance().Update(mode, t);
  }};
}

// This destructor gets called when the robot program shuts down.
// Cleanup any resources (especially files) before the robot code gets
// restarted.
Robot::~Robot() {}

void Robot::DisabledExit() {
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.has_value()) {
    Locations::GetInstance().Generate(alliance.value());
  }
}

#ifndef RUNNING_FRC_TESTS
int main(int argc, char **argv) { frc::StartRobot<Robot>(); }
#endif
