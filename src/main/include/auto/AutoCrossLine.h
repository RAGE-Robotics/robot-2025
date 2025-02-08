#pragma once

#include "TaskList.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>

class AutoCrossLine : public TaskList {
public:
  AutoCrossLine(frc::DriverStation::Alliance alliance, frc::Pose2d start);
};
