#pragma once

#include "Task.h"

#include "systems/Feeder.h"

class MoveFeeder : public Task {
public:
  void Start(double t) override;
  void Update(double t) override;
  void Stop() override;

  bool IsDone() const override;

  MoveFeeder(Feeder::Position position);

private:
  Feeder::Position m_position;
};
