#pragma once

#include <vector>

#include "Task.h"

// Runs all of the specified tasks in parallel
class ParallelTask : public Task {
public:
  ParallelTask(std::vector<Task> tasks = {});

  void Start(double t) override;
  void Update(double t) override;
  void Stop() override;

  bool IsDone() const override;

private:
  std::vector<Task> m_tasks;
};
