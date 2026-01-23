package frc.robot.subsystems.indexer;

import frc.robot.goals.RobotGoal;
import frc.robot.util.AllEvents;
import frc.robot.util.SubsystemBehavior;

public class IndexerBehavior extends SubsystemBehavior {
  private final IndexerSubsystem indexer;

  public IndexerBehavior(IndexerSubsystem indexer) {
    this.indexer = indexer;
  }

  @Override
  public void configure(AllEvents events) {
    events
        .goals()
        .isStateTrigger(RobotGoal.IDLE)
        .whileTrue(indexer.setStateCommand(IndexerState.IDLE));
    events
        .goals()
        .isStateTrigger(RobotGoal.LAUNCHING)
        .whileTrue(indexer.setStateCommand(IndexerState.FEEDING));
  }
}
