#pragma once

#include <frc/geometry/Pose2d.h>

#include "Constants.h"
#include "Task.h"
#include "control/PIDController.h"

class DriveLine : public Task {
  DriveLine(frc::Pose2d start, frc::Pose2d stop, bool resetPose = false,
            bool stopDistance = true, double maxV = Constants::kDefaultMaxV,
            double maxW = Constants::kDefaultMaxW,
            double positionTolerance = Constants::kDefaultPositionTolerance,
            double angleTolerance = Constants::kDefaultAngleTolerance);

  void Start(double t) override;
  void Update(double t) override;
  void Stop() override;

  bool IsDone() const override;

private:
  frc::Pose2d m_start, m_stop;
  bool m_resetPose, m_stopDistance;
  double m_maxV, m_maxW, m_positionTolerance, m_angleTolerance;

  PIDController m_pidX, m_pidY, m_pidAngle;
};
