package frc.robot.util;

import frc.robot.aiming.AimingEvents;
import frc.robot.goals.RobotEvents;
import frc.robot.power.PowerEvents;
import frc.robot.state.MatchEvents;
import frc.robot.subsystems.climber.ClimberEvents;
import frc.robot.subsystems.drive.DriveEvents;
import frc.robot.subsystems.hood.HoodEvents;
import frc.robot.subsystems.indexer.IndexerEvents;
import frc.robot.subsystems.intake.IntakeEvents;
import frc.robot.subsystems.shooter.ShooterEvents;
import frc.robot.subsystems.turret.TurretEvents;

// Hold all the events in one variable.
public record AllEvents(
    RobotEvents goals,
    MatchEvents match,
    IndexerEvents indexer,
    ShooterEvents shooter,
    TurretEvents turret,
    IntakeEvents intake,
    ClimberEvents climber,
    HoodEvents hood,
    DriveEvents drive,
    AimingEvents aiming,
    PowerEvents power) {}
