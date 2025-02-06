#include "auto/IntakeMode.h"

#include "systems/Intake.h"

void IntakeMode::Start(double t) {
  Intake::GetInstance().SetCoralState(m_coralState);
  Intake::GetInstance().SetAlgaeState(m_algaeState);
}

void IntakeMode::Update(double t) {}

void IntakeMode::Stop() {}

bool IntakeMode::IsDone() const { return true; }

IntakeMode::IntakeMode(Intake::CoralState coralState,
                       Intake::AlgaeState algaeState) {
  m_coralState = coralState;
  m_algaeState = algaeState;
}
