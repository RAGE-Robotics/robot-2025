#include "auto/ManipulatorIn.h"

#include "systems/Manipulator.h"

void ManipulatorIn::Start(double t) {
  Manipulator::GetInstance().StartIntakingCoral();
}

void ManipulatorIn::Update(double t) {
  if (IsDone()) {
    Manipulator::GetInstance().StopIntakingCoral();
  }
}

void ManipulatorIn::Stop() { Manipulator::GetInstance().StopIntakingCoral(); }

bool ManipulatorIn::IsDone() const {
  return Manipulator::GetInstance().DoneIntaking();
}

ManipulatorIn::ManipulatorIn() {}
