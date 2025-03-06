#pragma once

#include "TaskList.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>

class AutoOneCoral : public TaskList {
public:
  AutoOneCoral(frc::DriverStation::Alliance alliance, int position);
};
