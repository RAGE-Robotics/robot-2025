#include "systems/Cameras.h"

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonCamera.h>

#include "systems/SwerveDrive.h"

void Cameras::Update(Robot::Mode mode, double t) {
  // Front camera
  auto results = m_frontCamera.GetAllUnreadResults();
  if (results.size() > 0) {
    auto pose = m_frontPoseEstimator.Update(results[0]);

    if (pose.has_value()) {
      SwerveDrive::GetInstance().VisionUpdate(
          pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
    }
  }

  // Back camera
  results = m_backCamera.GetAllUnreadResults();
  if (results.size() > 0) {
    auto pose = m_backPoseEstimator.Update(results[0]);

    if (pose.has_value()) {
      SwerveDrive::GetInstance().VisionUpdate(
          pose.value().estimatedPose.ToPose2d(), pose.value().timestamp);
    }
  }

  results = m_detectCamera.GetAllUnreadResults();
  if (results.size() > 0 && results[0].HasTargets()) {
    if (results[0].GetTargets().size() > 1) {
      m_algaeState = kBoth;
    } else {
      if (results[0].GetTargets()[0].GetPitch() > 0) {
        m_algaeState = kTop;
      } else {
        m_algaeState = kBottom;
      }
    }
  } else {
    m_algaeState = kNone;
  }
}

Cameras::AlgaeState Cameras::GetAlgaeState() const { return m_algaeState; }

Cameras::Cameras() {}
