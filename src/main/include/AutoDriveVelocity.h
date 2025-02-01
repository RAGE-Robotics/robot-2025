#pragma once

#include "Task.h"

class AutoDriveVelocity : public Task {
public:
  AutoDriveVelocity(double vx = 0, double vy = 0, double w = 0);

  void Start(double t) override;
  void Update(double t) override;
  void Stop() override;

  bool IsDone() const override;

private:
  double m_vx, m_vy, m_w;
};
