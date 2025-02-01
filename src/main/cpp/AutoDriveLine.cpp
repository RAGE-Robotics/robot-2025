#include "AutoDriveLine.h"
#include "SwerveDrive.h"
#include "Util.h"

AutoDriveLine::AutoDriveLine(frc::Pose2d start, frc::Pose2d stop, double maxV,
                             double maxW, double positionTolerance,
                             double angleTolerance)
    : m_start{start}, m_stop{stop} {
  m_maxV = maxV;
  m_maxW = maxW;
  m_positionTolerance = positionTolerance;
  m_angleTolerance = angleTolerance;
}

void AutoDriveLine::Start(double t) {}

void AutoDriveLine::Update(double t) {}

void AutoDriveLine::Stop() {}

bool AutoDriveLine::IsDone() const {
  return SwerveDrive::GetInstance()
                 .GetPose2d()
                 .Translation()
                 .Distance(m_stop.Translation())
                 .value() <= m_positionTolerance &&
         Util::AngleDistance(SwerveDrive::GetInstance().GetPose2d().Rotation(),
                             m_stop.Rotation()) <= m_angleTolerance;
}
