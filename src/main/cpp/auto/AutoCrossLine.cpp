#include "auto/AutoCrossLine.h"

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>

#include "auto/DriveLine.h"
#include "auto/MoveFeeder.h"
#include "frc/DriverStation.h"

#include "systems/Feeder.h"

AutoCrossLine::AutoCrossLine(frc::DriverStation::Alliance alliance,
                             frc::Pose2d start)
    : TaskList{{MoveFeeder{Feeder::Position::kUp},
                DriveLine{
                    start,
                    frc::Pose2d{
                        frc::Translation2d{
                            units::meter_t{
                                start.Translation().X().value() +
                                (alliance == frc::DriverStation::Alliance::kBlue
                                     ? -1
                                     : 1)},
                            start.Translation().Y()},
                        start.Rotation()}}}} {}
