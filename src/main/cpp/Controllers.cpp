#include "Controllers.h"

Controllers::Controllers() : m_driverController{0}, m_operatorController{1} {}

frc::XboxController &Controllers::GetDriverController() {
  return m_driverController;
}

frc::XboxController &Controllers::GetOperatorController() {
  return m_operatorController;
}
