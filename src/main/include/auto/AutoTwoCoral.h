#pragma once

#include "TaskList.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>

class AutoTwoCoral : public TaskList {
public:
  AutoTwoCoral(frc::DriverStation::Alliance alliance, int position);
};
