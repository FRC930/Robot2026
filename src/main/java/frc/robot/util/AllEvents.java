package frc.robot.util;

import frc.robot.aiming.AimingEvents;
import frc.robot.goals.RobotEvents;
import frc.robot.state.MatchEvents;
import frc.robot.subsystems.climber.ClimberEvents;
import frc.robot.subsystems.compactor.CompactorEvents;
import frc.robot.subsystems.drive.DriveEvents;
import frc.robot.subsystems.feeder.FeederEvents;
import frc.robot.subsystems.hood.HoodEvents;
import frc.robot.subsystems.indexer.IndexerEvents;
import frc.robot.subsystems.intake.IntakeEvents;
import frc.robot.subsystems.shooter.ShooterEvents;

// Hold all the events in one variable.
public record AllEvents(
    RobotEvents goals,
    MatchEvents match,
    IndexerEvents indexer,
    FeederEvents feeder,
    ShooterEvents shooter,
    IntakeEvents intake,
    ClimberEvents climber,
    HoodEvents hood,
    CompactorEvents compactor,
    DriveEvents drive,
    AimingEvents aiming) {}
