package frc.robot.util;

import frc.robot.goals.RobotEvents;
import frc.robot.state.MatchEvents;
import frc.robot.subsystems.indexer.IndexerEvents;

public record AllEvents(RobotEvents goals, MatchEvents match, IndexerEvents indexer) {}
