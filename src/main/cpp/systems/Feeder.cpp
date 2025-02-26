#include "systems/Feeder.h"

#include "Constants.h"

void Feeder::SetPosition(Position position) { m_position = position; }

void Feeder::Update(Robot::Mode mode, double t) {
  if (m_position == kUp) {
    m_solenoid.Set(true);
  } else {
    m_solenoid.Set(false);
  }
}

Feeder::Feeder()
    : m_solenoid(frc::PneumaticsModuleType::REVPH,
                 Constants::kFeederSolenoidId) {}
