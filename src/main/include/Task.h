#pragma once

#include "Robot.h"

class Task {
public:
  virtual void Start(double t);
  virtual void Update(double t);
  virtual void Stop(double t);

  virtual bool IsDone() const;
};
