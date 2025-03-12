#include "auto/AutoOneCoral.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <memory>

#include "Locations.h"
#include "auto/Delay.h"
#include "auto/FollowPath.h"
#include "auto/ManipulatorOut.h"
#include "auto/MoveElevator.h"
#include "systems/Elevator.h"

AutoOneCoral::AutoOneCoral(frc::DriverStation::Alliance alliance,
                           int position) {
  int setpoint = 6;
  if (alliance == frc::DriverStation::Alliance::kRed) {
    if (position == 1) {
      setpoint = 9;
    } else if (position == 3) {
      setpoint = 4;
    }
  } else {
    if (position == 1) {
      setpoint = 4;
    } else if (position == 3) {
      setpoint = 9;
    }
  }

  m_tasks.push_back(std::make_shared<FollowPath>(
      std::vector<frc::Pose2d>{
          Locations::GetInstance().GetStartPosition(alliance, position),
          Locations::GetInstance().GetCoralPositions()[setpoint]},
      false, false));
  m_tasks.push_back(std::make_shared<MoveElevator>(Elevator::kL4));
  m_tasks.push_back(std::make_shared<Delay>(3));
  m_tasks.push_back(std::make_shared<ManipulatorOut>());
  m_tasks.push_back(std::make_shared<Delay>(1));
  m_tasks.push_back(std::make_shared<ManipulatorOut>(true));
  m_tasks.push_back(std::make_shared<MoveElevator>(Elevator::kHome));
  m_tasks.push_back(std::make_shared<Delay>(3));
}
