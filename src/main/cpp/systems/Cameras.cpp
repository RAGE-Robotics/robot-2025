#include "systems/Cameras.h"

#include <algorithm>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonCamera.h>

#include "systems/SwerveDrive.h"

void Cameras::Update(Robot::Mode mode, double t) {
  // Front camera
  auto results = m_frontCamera.GetAllUnreadResults();

  // See
  // https://stackoverflow.com/questions/1380463/how-do-i-sort-a-vector-of-custom-objects
  std::sort(results.begin(), results.end(), [](auto left, auto right) {
    return left.GetTimestamp() < right.GetTimestamp();
  });

  for (auto result : results) {
    auto pose = m_frontPoseEstimator.Update(result);

    if (pose.has_value()) {
      SwerveDrive::GetInstance().VisionUpdate(
          pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
    }
  }

  // Back camera
  results = m_backCamera.GetAllUnreadResults();

  std::sort(results.begin(), results.end(), [](auto left, auto right) {
    return left.GetTimestamp() < right.GetTimestamp();
  });

  for (auto result : results) {
    auto pose = m_backPoseEstimator.Update(result);

    if (pose.has_value()) {
      SwerveDrive::GetInstance().VisionUpdate(
          pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
    }
  }
}

Cameras::AlgaeState Cameras::GetAlgaeState() const { return m_algaeState; }

Cameras::Cameras() {}
