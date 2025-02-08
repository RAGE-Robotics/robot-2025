#include "auto/ManipulatorMode.h"

#include "systems/Manipulator.h"

void ManipulatorMode::Start(double t) {
  Manipulator::GetInstance().SetCoralState(m_coralState);
  Manipulator::GetInstance().SetAlgaeState(m_algaeState);
}

void ManipulatorMode::Update(double t) {}

void ManipulatorMode::Stop() {}

bool ManipulatorMode::IsDone() const { return true; }

ManipulatorMode::ManipulatorMode(Manipulator::CoralState coralState,
                                 Manipulator::AlgaeState algaeState) {
  m_coralState = coralState;
  m_algaeState = algaeState;
}
