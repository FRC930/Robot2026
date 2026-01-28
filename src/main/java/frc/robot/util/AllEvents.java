package frc.robot.util;

import frc.robot.goals.RobotEvents;
import frc.robot.state.MatchEvents;
import frc.robot.subsystems.climber.ClimberEvents;
import frc.robot.subsystems.indexer.IndexerEvents;
import frc.robot.subsystems.intake.IntakeEvents;
import frc.robot.subsystems.shooter.ShooterEvents;

// Hold all the events in one variable.
public record AllEvents(
    RobotEvents goals,
    MatchEvents match,
    IndexerEvents indexer,
    ShooterEvents shooter,
    IntakeEvents intake,
    ClimberEvents climber) {}
// (:
