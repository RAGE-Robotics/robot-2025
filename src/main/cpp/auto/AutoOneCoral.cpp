#include "auto/AutoOneCoral.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <memory>

#include "Locations.h"
#include "auto/FollowPath.h"
#include "auto/MoveElevator.h"
#include "frc/geometry/Rotation2d.h"
#include "systems/Elevator.h"

AutoOneCoral::AutoOneCoral(frc::DriverStation::Alliance alliance,
                           int position) {
  m_tasks.push_back(std::make_shared<FollowPath>(
      std::vector<frc::Pose2d>{
          Locations::GetInstance().GetStartPosition(alliance, position),
          Locations::GetInstance()
              .GetCoralPositions()[alliance == frc::DriverStation::kBlue ? 0
                                                                         : 11]},
      false, false));
  m_tasks.push_back(std::make_shared<MoveElevator>(Elevator::kL2));
}
