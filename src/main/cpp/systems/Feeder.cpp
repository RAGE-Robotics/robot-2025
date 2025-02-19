#include "systems/Feeder.h"

#include "Constants.h"

void Feeder::SetPosition(Position position) { m_position = position; }

void Feeder::Update(Robot::Mode mode, double t) {
  if (m_position == kUp) {
    m_solenoid.Set(frc::DoubleSolenoid::kForward);
  } else {
    m_solenoid.Set(frc::DoubleSolenoid::kReverse);
  }
}

Feeder::Feeder()
    : m_solenoid(frc::PneumaticsModuleType::REVPH,
                 Constants::kFeederSolenoidIdForward,
                 Constants::kFeederSolenoidIdReverse) {}
