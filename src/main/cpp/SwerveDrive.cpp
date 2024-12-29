#include "SwerveDrive.h"

#include "Constants.h"
#include "Controllers.h"
#include "Robot.h"

// We need to initialize the gyro and kinematics members. The kinematics
// constructor needs the positions of the four wheels. The coordinate system is
// +x is towards the front of the robot, and +y is to the robot's left.
SwerveDrive::SwerveDrive()
    : m_gyro{Constants::kPigeonCanId},
      m_kinematics{frc::Translation2d{Constants::kWheelBaseWidth / 2,
                                      Constants::kWheelBaseWidth / 2},
                   frc::Translation2d{Constants::kWheelBaseWidth / 2,
                                      -Constants::kWheelBaseWidth / 2},
                   frc::Translation2d{-Constants::kWheelBaseWidth / 2,
                                      Constants::kWheelBaseWidth / 2},
                   frc::Translation2d{-Constants::kWheelBaseWidth / 2,
                                      -Constants::kWheelBaseWidth / 2}},
      m_steeringMotors{{0}, {1}, {2}, {4}}, m_driveMotors{{5}, {6}, {7}, {8}} {
  // Configure the PID values for the position mode on the steering motors
  auto [kS, kV, kP, kI, kD] = Constants::kSteeringMotorGains;
  configs::Slot0Configs config;
  config.kS = kS;
  config.kV = kV;
  config.kP = kP;
  config.kI = kI;
  config.kD = kD;
  for (int i = 0; i < 4; i++) {
    m_steeringMotors[i].GetConfigurator().Apply(config);
  }
}

// This function needs to be called by the looper to update the drive motors
void SwerveDrive::Update(Robot::Mode mode) {
  if (mode == Robot::kTeleop) {
    double x = -Controllers::GetInstance().GetDriverController().GetLeftY();
    double y = -Controllers::GetInstance().GetDriverController().GetLeftX();
    double rotation =
        Controllers::GetInstance().GetDriverController().GetRightX();
    auto [fl, fr, bl, br] =
        m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds{
            units::meters_per_second_t{x}, units::meters_per_second_t{y},
            units::radians_per_second_t{rotation}});

    m_steeringMotors[0].SetControl(controls::VelocityVoltage{
        units::angular_velocity::turns_per_second_t{
            fl.angle.Radians().value() / 2 /
            M_PI}}.WithSlot(0));
    m_steeringMotors[1].SetControl(controls::VelocityVoltage{
        units::angular_velocity::turns_per_second_t{
            fr.angle.Radians().value() / 2 /
            M_PI}}.WithSlot(0));
    m_steeringMotors[2].SetControl(controls::VelocityVoltage{
        units::angular_velocity::turns_per_second_t{
            bl.angle.Radians().value() / 2 /
            M_PI}}.WithSlot(0));
    m_steeringMotors[3].SetControl(controls::VelocityVoltage{
        units::angular_velocity::turns_per_second_t{
            br.angle.Radians().value() / 2 /
            M_PI}}.WithSlot(0));
  }
}

frc::Rotation2d SwerveDrive::GetGyroRotation2d() const {
  return m_gyro.GetRotation2d();
}
