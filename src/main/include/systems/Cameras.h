#pragma once

#include <memory>
#include <photon/PhotonPoseEstimator.h>

#include "Constants.h"
#include "System.h"

class Cameras : public System {
public:
  static Cameras &GetInstance() {
    static Cameras instance;
    return instance;
  }

  void Update(Robot::Mode mode, double t) override;

private:
  photon::PhotonCamera m_frontCamera{"FRONT"};
  photon::PhotonCamera m_backCamera{"BACK"};

  photon::PhotonPoseEstimator m_frontPoseEstimator{
      frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025Reefscape),
      photon::MULTI_TAG_PNP_ON_COPROCESSOR, Constants::kFrontCameraTransform};
  photon::PhotonPoseEstimator m_backPoseEstimator{
      frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025Reefscape),
      photon::MULTI_TAG_PNP_ON_COPROCESSOR, Constants::kBackCameraTransform};

  Cameras();
};
