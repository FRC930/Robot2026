package frc.robot.util;

import frc.robot.goals.RobotGoal;
import frc.robot.state.MatchStates;
import frc.robot.subsystems.indexer.IndexerState;

public record AllEvents(
    StateSubsystem<RobotGoal> goals,
    StateSubsystem<MatchStates> match,
    StateSubsystem<IndexerState> indexer) {}
