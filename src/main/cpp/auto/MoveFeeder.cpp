#include "auto/MoveFeeder.h"

void MoveFeeder::Start(double t) {
  Feeder::GetInstance().SetPosition(m_position);
}

void MoveFeeder::Update(double t) {}

void MoveFeeder::Stop() {}

bool MoveFeeder::IsDone() const { return true; }

MoveFeeder::MoveFeeder(Feeder::Position position) { m_position = position; }
