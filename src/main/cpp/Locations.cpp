#include "Locations.h"

#include <cmath>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include "Constants.h"

void Locations::Generate(frc::DriverStation::Alliance alliance) {
  m_coralPositions.clear();
  m_algaePositions.clear();

  for (int i = 0; i < 6; i++) {
    double angle = 2 * M_PI * (i / 6.0) +
                   (alliance == frc::DriverStation::Alliance::kBlue ? M_PI : 0);

    double reef = alliance == frc::DriverStation::Alliance::kRed
                      ? Constants::kFieldLength - Constants::kReefOffset
                      : Constants::kReefOffset;
    double centerDistance = Constants::kReefWidth / 2 + Constants::kReefSpacing;

    m_algaePositions.push_back(
        frc::Pose2d{frc::Translation2d{
                        units::meter_t{reef + std::cos(angle) * centerDistance},
                        units::meter_t{Constants::kFieldWidth / 2 +
                                       std::sin(angle) * centerDistance}},
                    frc::Rotation2d{units::radian_t{angle + M_PI}}});

    m_coralPositions.push_back(frc::Pose2d{
        frc::Translation2d{
            units::meter_t{reef + std::cos(angle) * centerDistance +
                           (Constants::kCoralSpacing / 2) *
                               std::cos(angle - M_PI / 2)},
            units::meter_t{
                Constants::kFieldWidth / 2 + std::sin(angle) * centerDistance +
                +(Constants::kCoralSpacing / 2) * std::sin(angle - M_PI / 2)}},
        frc::Rotation2d{units::radian_t{angle + M_PI}}});
    m_coralPositions.push_back(frc::Pose2d{
        frc::Translation2d{
            units::meter_t{reef + std::cos(angle) * centerDistance +
                           (Constants::kCoralSpacing / 2) *
                               std::cos(angle + M_PI / 2)},
            units::meter_t{
                Constants::kFieldWidth / 2 + std::sin(angle) * centerDistance +
                +(Constants::kCoralSpacing / 2) * std::sin(angle + M_PI / 2)}},
        frc::Rotation2d{units::radian_t{angle + M_PI}}});
  }
}

const std::vector<frc::Pose2d> Locations::GetCoralPositions() const {
  return m_coralPositions;
}

const std::vector<frc::Pose2d> Locations::GetAlgaePositions() const {
  return m_algaePositions;
}

frc::Pose2d Locations::GetStartPosition(frc::DriverStation::Alliance alliance,
                                        int i) const {
  double x = alliance == frc::DriverStation::Alliance::kRed
                 ? Constants::kFieldLength - Constants::kStartLineOffset +
                       Constants::kStartOffsetX
                 : Constants::kStartLineOffset - Constants::kStartOffsetX;
  double y = 0;
  switch (i) {
  case 1:
    y += alliance == frc::DriverStation::Alliance::kRed
             ? -Constants::kStartOffsetY
             : Constants::kStartOffsetY;
    break;
  case 3:
    y -= alliance == frc::DriverStation::Alliance::kRed
             ? -Constants::kStartOffsetY
             : Constants::kStartOffsetY;
    break;
  default:
    break;
  }

  double angle = alliance == frc::DriverStation::Alliance::kRed ? 0 : M_PI;
  return frc::Pose2d{frc::Translation2d{units::meter_t{x}, units::meter_t{y}},
                     frc::Rotation2d{units::radian_t{angle}}};
}

Locations::Locations() {}
