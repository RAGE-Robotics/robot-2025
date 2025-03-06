#include "auto/AutoOneCoral.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <memory>

#include "Locations.h"
#include "auto/FollowPath.h"

AutoOneCoral::AutoOneCoral(frc::DriverStation::Alliance alliance,
                           int position) {
  m_tasks.push_back(std::make_shared<FollowPath>(
      std::vector<frc::Pose2d>{
          Locations::GetInstance().GetStartPosition(alliance, position),
          Locations::GetInstance().GetCoralPositions()[6]},
      false, false));
}
