#include "auto/AutoTwoCoral.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <memory>

#include "Locations.h"
#include "auto/Delay.h"
#include "auto/FollowPath.h"
#include "auto/ManipulatorIn.h"
#include "auto/ManipulatorOut.h"
#include "auto/MoveElevator.h"
#include "systems/Elevator.h"

AutoTwoCoral::AutoTwoCoral(frc::DriverStation::Alliance alliance,
                           int position) {
  int setpoint = 6;
  if (position == 1) {
    setpoint = 9;
  } else if (position == 3) {
    setpoint = 4;
  }

  m_tasks.push_back(std::make_shared<FollowPath>(
      std::vector<frc::Pose2d>{
          Locations::GetInstance().GetStartPosition(alliance, position),
          Locations::GetInstance().GetCoralPositions()[setpoint]},
      false, false));
  m_tasks.push_back(std::make_shared<MoveElevator>(Elevator::kL4));
  m_tasks.push_back(std::make_shared<Delay>(2));
  m_tasks.push_back(std::make_shared<ManipulatorOut>());
  m_tasks.push_back(std::make_shared<Delay>(1));
  m_tasks.push_back(std::make_shared<ManipulatorOut>(true));
  m_tasks.push_back(std::make_shared<MoveElevator>(Elevator::kHome));

  if (position == 2) {
    m_tasks.push_back(std::make_shared<Delay>(2));
  } else {
    int feeder_index;
    if (alliance == frc::DriverStation::Alliance::kBlue) {
      if (position == 1) {
        feeder_index = 1;
      } else if (position == 3) {
        feeder_index = 0;
      }
    } else {
      if (position == 1) {
        feeder_index = 2;
      } else if (position == 3) {
        feeder_index = 3;
      }
    }

    m_tasks.push_back(std::make_shared<FollowPath>(
        std::vector<frc::Pose2d>{
            Locations::GetInstance().GetCoralPositions()[setpoint],
            Locations::GetInstance().GetFeederPositions()[feeder_index]},
        false, false));
    m_tasks.push_back(std::make_shared<ManipulatorIn>());

    if (position == 1) {
      setpoint = 11;
    } else if (position == 3) {
      setpoint = 2;
    }

    m_tasks.push_back(std::make_shared<FollowPath>(
        std::vector<frc::Pose2d>{
            Locations::GetInstance().GetFeederPositions()[feeder_index],
            Locations::GetInstance().GetCoralPositions()[setpoint]},
        false, false));
    m_tasks.push_back(std::make_shared<MoveElevator>(Elevator::kL4));
    m_tasks.push_back(std::make_shared<Delay>(2));
    m_tasks.push_back(std::make_shared<ManipulatorOut>());
    m_tasks.push_back(std::make_shared<Delay>(1));
    m_tasks.push_back(std::make_shared<ManipulatorOut>(true));
    m_tasks.push_back(std::make_shared<MoveElevator>(Elevator::kHome));
    m_tasks.push_back(std::make_shared<Delay>(2));
  }
}
