package frc.robot.subsystems.indexer;

import frc.robot.util.AllEvents;
import frc.robot.util.SubsystemBehavior;

public class IndexerBehavior extends SubsystemBehavior {
  private final IndexerSubsystem indexer;

  public IndexerBehavior(IndexerSubsystem indexer) {
    this.indexer = indexer;
  }

  @Override
  public void configure(AllEvents events) {
    events.goals().isIdleTrigger().whileTrue(indexer.idleCommand());
    events.goals().isShootingTrigger().whileTrue(indexer.indexingCommand());
  }
}
