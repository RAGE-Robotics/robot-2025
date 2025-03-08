#include "auto/ManipulatorOut.h"

#include "systems/Manipulator.h"

void ManipulatorOut::Start(double t) {
  if (m_stop) {
    Manipulator::GetInstance().StopOutakingCoral();
  } else {
    Manipulator::GetInstance().StartOutakingCoral();
  }

  m_done = true;
}

void ManipulatorOut::Update(double t) {}

void ManipulatorOut::Stop() {}

bool ManipulatorOut::IsDone() const { return m_done; }

ManipulatorOut::ManipulatorOut(bool stop) : m_stop{stop} {}
