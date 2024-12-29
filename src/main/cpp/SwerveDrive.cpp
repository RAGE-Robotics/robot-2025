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
      m_steeringMotors{{Constants::kFlSteeringMotorId},
                       {Constants::kFrSteeringMotorId},
                       {Constants::kBlSteeringMotorId},
                       {Constants::kBrSteeringMotorId}},
      m_driveMotors{{Constants::kFlDriveMotorId},
                    {Constants::kFrDriveMotorId},
                    {Constants::kBlDriveMotorId},
                    {Constants::kBrDriveMotorId}},
      m_encoders{{Constants::kFlEncoderId},
                 {Constants::kFrEncoderId},
                 {Constants::kBlEncoderId},
                 {Constants::kBrEncoderId}},
      m_poseEstimator{
          m_kinematics,
          GetGyroRotation2d(),
          {frc::SwerveModulePosition{
               units::meter_t{
                   m_driveMotors[0].GetPosition().GetValue().value() * M_PI *
                   Constants::kWheelRadius},
               frc::Rotation2d{m_encoders[0].GetPosition().GetValue() * 2 *
                               M_PI}},
           frc::SwerveModulePosition{
               units::meter_t{
                   m_driveMotors[1].GetPosition().GetValue().value() * M_PI *
                   Constants::kWheelRadius},
               frc::Rotation2d{m_encoders[1].GetPosition().GetValue() * 2 *
                               M_PI}},
           frc::SwerveModulePosition{
               units::meter_t{
                   m_driveMotors[2].GetPosition().GetValue().value() * M_PI *
                   Constants::kWheelRadius},
               frc::Rotation2d{m_encoders[2].GetPosition().GetValue() * 2 *
                               M_PI}},
           frc::SwerveModulePosition{
               units::meter_t{
                   m_driveMotors[3].GetPosition().GetValue().value() * M_PI *
                   Constants::kWheelRadius},
               frc::Rotation2d{m_encoders[3].GetPosition().GetValue() * 2 *
                               M_PI}}},
          frc::Pose2d{}} {
  // Configure the PID values for the position mode on the steering motors
  auto [kS, kV, kP, kI, kD] = Constants::kSteeringMotorGains;
  configs::Slot0Configs config;
  config.kS = kS;
  config.kV = kV;
  config.kP = kP;
  config.kI = kI;
  config.kD = kD;

  // At the same time, go ahead and configur the remote sensor to be the
  // CANCoder.
  configs::TalonFXConfiguration talonConfig;
  talonConfig.Feedback.FeedbackSensorSource =
      signals::FeedbackSensorSourceValue::RemoteCANcoder;

  // Create a current limit config to apply to the drive motors
  auto currentLimitConfig = configs::CurrentLimitsConfigs{}
                                .WithStatorCurrentLimitEnable(true)
                                .WithStatorCurrentLimit(units::ampere_t{
                                    Constants::kDriveCurrentLimit});

  // Apply an open loop ramp rate to the drive motors only
  auto rampRateConfig =
      configs::OpenLoopRampsConfigs{}.WithVoltageOpenLoopRampPeriod(
          units::second_t{Constants::kDriveRampRate});

  for (int i = 0; i < 4; i++) {
    m_steeringMotors[i].GetConfigurator().Apply(config);

    talonConfig.Feedback.FeedbackRemoteSensorID = m_encoders[i].GetDeviceID();
    m_steeringMotors[i].GetConfigurator().Apply(talonConfig);

    m_driveMotors[i].GetConfigurator().Apply(currentLimitConfig);
    m_driveMotors[i].GetConfigurator().Apply(rampRateConfig);
  }
}

// This function needs to be called by the looper to update the drive motors
void SwerveDrive::Update(Robot::Mode mode) {
  // Update the estimation of where the robot thinks it is on the field
  m_poseEstimator.Update(
      GetGyroRotation2d(),
      {frc::SwerveModulePosition{
           units::meter_t{m_driveMotors[0].GetPosition().GetValue().value() *
                          M_PI * Constants::kWheelRadius},
           frc::Rotation2d{m_encoders[0].GetPosition().GetValue() * 2 * M_PI}},
       frc::SwerveModulePosition{
           units::meter_t{m_driveMotors[1].GetPosition().GetValue().value() *
                          M_PI * Constants::kWheelRadius},
           frc::Rotation2d{m_encoders[1].GetPosition().GetValue() * 2 * M_PI}},
       frc::SwerveModulePosition{
           units::meter_t{m_driveMotors[2].GetPosition().GetValue().value() *
                          M_PI * Constants::kWheelRadius},
           frc::Rotation2d{m_encoders[2].GetPosition().GetValue() * 2 * M_PI}},
       frc::SwerveModulePosition{
           units::meter_t{m_driveMotors[3].GetPosition().GetValue().value() *
                          M_PI * Constants::kWheelRadius},
           frc::Rotation2d{m_encoders[3].GetPosition().GetValue() * 2 *
                           M_PI}}});

  if (mode == Robot::kTeleop) {
    // Get the inputs from the controller. Note this uses the split setup where
    // the left joystick controls velocity, and the right joystick controls the
    // rotation.
    double x = -Controllers::GetInstance().GetDriverController().GetLeftY();
    double y = -Controllers::GetInstance().GetDriverController().GetLeftX();
    double rotation =
        Controllers::GetInstance().GetDriverController().GetRightX();

    // Use the WPILib kinematics class to determine the individual wheel angles
    // and velocities.
    auto speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        units::meters_per_second_t{x}, units::meters_per_second_t{y},
        units::radians_per_second_t{rotation}, GetGyroRotation2d());
    auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds);

    // Optimize the angle setpoints to make the wheels reach the correct angle
    // as fast as possible (not go the long way around).
    fl.Optimize(m_encoders[0].GetPosition().GetValue() * 2 * M_PI);
    fr.Optimize(m_encoders[1].GetPosition().GetValue() * 2 * M_PI);
    bl.Optimize(m_encoders[2].GetPosition().GetValue() * 2 * M_PI);
    br.Optimize(m_encoders[3].GetPosition().GetValue() * 2 * M_PI);

    // Decrease the speed of modules that aren't pointing in the correct
    // direction.
    fl.speed *= (fl.angle - frc::Rotation2d{units::radian_t{
                                m_encoders[0].GetPosition().GetValue()}})
                    .Cos();
    fr.speed *= (fr.angle - frc::Rotation2d{units::radian_t{
                                m_encoders[1].GetPosition().GetValue()}})
                    .Cos();
    bl.speed *= (bl.angle - frc::Rotation2d{units::radian_t{
                                m_encoders[2].GetPosition().GetValue()}})
                    .Cos();
    br.speed *= (br.angle - frc::Rotation2d{units::radian_t{
                                m_encoders[3].GetPosition().GetValue()}})
                    .Cos();

    // Set the positions for the wheel angles
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

frc::Pose2d SwerveDrive::GetPose2d() const {
  return m_poseEstimator.GetEstimatedPosition();
}
