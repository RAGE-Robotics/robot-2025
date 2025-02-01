#pragma once

#include <vector>

#include "Task.h"

// Runs all of the tasks specified sequentially
class TaskList : public Task {
public:
  TaskList(std::vector<Task> tasks = {});

  void Start(double t) override;
  void Update(double t) override;
  void Stop() override;

  bool IsDone() const override;

private:
  std::vector<Task> m_tasks;
  int m_taskIndex = 0;
  bool m_done = false;
};
