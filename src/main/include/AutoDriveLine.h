#pragma once

#include <frc/geometry/Pose2d.h>

#include "Constants.h"
#include "Task.h"

class AutoDriveLine : public Task {
  AutoDriveLine(frc::Pose2d start, frc::Pose2d stop,
                double maxV = Constants::kDefaultMaxV,
                double maxW = Constants::kDefaultMaxW,
                double positionTolerance = Constants::kDefaultPositionTolerance,
                double angleTolerance = Constants::kDefaultAngleTolerance);

  void Start(double t) override;
  void Update(double t) override;
  void Stop() override;

  bool IsDone() const override;

private:
  frc::Pose2d m_start, m_stop;
  double m_maxV, m_maxW, m_positionTolerance, m_angleTolerance;
};
