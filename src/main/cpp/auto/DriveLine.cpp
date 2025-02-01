#include "auto/DriveLine.h"

#include <algorithm>

#include "Util.h"
#include "systems/SwerveDrive.h"

DriveLine::DriveLine(frc::Pose2d start, frc::Pose2d stop, bool resetPose,
                     double maxV, double maxW, double positionTolerance,
                     double angleTolerance)
    : m_start{start}, m_stop{stop},
      m_pidX{Constants::kPathFollowingKp, Constants::kPathFollowingKi,
             Constants::kPathFollowingKd},
      m_pidY{Constants::kPathFollowingKp, Constants::kPathFollowingKi,
             Constants::kPathFollowingKd},
      m_pidAngle{Constants::kPathFollowingAngleKp,
                 Constants::kPathFollowingAngleKi,
                 Constants::kPathFollowingAngleKd} {
  m_resetPose = resetPose;
  m_maxV = maxV;
  m_maxW = maxW;
  m_positionTolerance = positionTolerance;
  m_angleTolerance = angleTolerance;
}

void DriveLine::Start(double t) {
  if (m_resetPose) {
    SwerveDrive::GetInstance().ResetPose(m_start);
  }
}

void DriveLine::Update(double t) {
  double vx = m_pidX.Update(
      SwerveDrive::GetInstance().GetPose2d().Translation().X().value(),
      m_stop.Translation().X().value());
  double vy = m_pidY.Update(
      SwerveDrive::GetInstance().GetPose2d().Translation().Y().value(),
      m_stop.Translation().Y().value());

  double angleSetpoint = m_stop.Rotation().Radians().value();
  double angleError =
      angleSetpoint -
      SwerveDrive::GetInstance().GetPose2d().Rotation().Radians().value();
  if (angleError > M_PI) {
    angleSetpoint -= 2 * M_PI;
  }
  double w = m_pidAngle.Update(
      SwerveDrive::GetInstance().GetPose2d().Rotation().Radians().value(),
      angleSetpoint);

  vx = std::clamp(vx, -m_maxV, m_maxV);
  vy = std::clamp(vy, -m_maxV, m_maxV);
  w = std::clamp(w, -m_maxW, m_maxW);

  SwerveDrive::GetInstance().DriveVelocity(vx, vy, w);
}

void DriveLine::Stop() { SwerveDrive::GetInstance().DriveVelocity(); }

bool DriveLine::IsDone() const {
  return SwerveDrive::GetInstance()
                 .GetPose2d()
                 .Translation()
                 .Distance(m_stop.Translation())
                 .value() <= m_positionTolerance &&
         Util::AngleDistance(SwerveDrive::GetInstance().GetPose2d().Rotation(),
                             m_stop.Rotation()) <= m_angleTolerance;
}
