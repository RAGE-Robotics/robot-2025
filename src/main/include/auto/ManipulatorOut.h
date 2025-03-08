#pragma once

#include "Constants.h"
#include "Task.h"
#include "systems/Manipulator.h"

class ManipulatorOut : public Task {
public:
  void Start(double t) override;
  void Update(double t) override;
  void Stop() override;

  bool IsDone() const override;

  ManipulatorOut(bool stop = false);

private:
  bool m_stop;
  bool m_done = false;
};
