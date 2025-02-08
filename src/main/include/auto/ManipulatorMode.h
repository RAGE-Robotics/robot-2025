#pragma once

#include "Task.h"
#include "systems/Manipulator.h"

class ManipulatorMode : public Task {
public:
  void Start(double t) override;
  void Update(double t) override;
  void Stop() override;

  bool IsDone() const override;

  ManipulatorMode(Manipulator::CoralState coralState,
                  Manipulator::AlgaeState algaeState);

private:
  Manipulator::CoralState m_coralState;
  Manipulator::AlgaeState m_algaeState;
};
