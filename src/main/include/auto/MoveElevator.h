#pragma once

#include "Constants.h"
#include "Task.h"
#include "systems/Elevator.h"

class MoveElevator : public Task {
public:
  void Start(double t) override;
  void Update(double t) override;
  void Stop() override;

  bool IsDone() const override;

  MoveElevator(Elevator::Position position,
               double tolerance = Constants::kElevatorDefaultTolerance);

private:
  Elevator::Position m_position;
  double m_tolerance;
};
