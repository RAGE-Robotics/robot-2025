#include "AutoDriveVelocity.h"
#include "SwerveDrive.h"

AutoDriveVelocity::AutoDriveVelocity(double vx, double vy, double w)
    : m_vx{vx}, m_vy{vy}, m_w{w} {}

void AutoDriveVelocity::Start(double t) {
  SwerveDrive::GetInstance().DriveVelocity(m_vx, m_vy, m_w);
}

void AutoDriveVelocity::Update(double t) {}

void AutoDriveVelocity::Stop() {}

bool AutoDriveVelocity::IsDone() const { return true; }
