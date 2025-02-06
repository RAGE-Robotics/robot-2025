#pragma once

#include "Task.h"
#include "systems/Intake.h"

class IntakeMode : public Task {
public:
  void Start(double t) override;
  void Update(double t) override;
  void Stop() override;

  bool IsDone() const override;

  IntakeMode(Intake::CoralState coralState, Intake::AlgaeState algaeState);

private:
  Intake::CoralState m_coralState;
  Intake::AlgaeState m_algaeState;
};
