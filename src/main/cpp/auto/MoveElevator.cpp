#include "auto/MoveElevator.h"

#include "systems/Elevator.h"

void MoveElevator::Start(double t) {
  Elevator::GetInstance().SetPosition(m_position);
}

void MoveElevator::Update(double t) {}

void MoveElevator::Stop() {}

bool MoveElevator::IsDone() const { return true; }

MoveElevator::MoveElevator(Elevator::Position position, double tolerance) {
  m_position = position;
  m_tolerance = tolerance;
}
