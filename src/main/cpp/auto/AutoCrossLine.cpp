#include "auto/AutoCrossLine.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>

#include "auto/Delay.h"
#include "auto/DriveVelocity.h"

AutoCrossLine::AutoCrossLine() {
  m_tasks.push_back(std::make_shared<DriveVelocity>(-1, 0, 0));
  m_tasks.push_back(std::make_shared<Delay>(1));
  m_tasks.push_back(std::make_shared<DriveVelocity>());
}
