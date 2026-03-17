package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Commands;
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

    events.goals().isReverseIndexer().whileTrue(indexer.reverseCommand());

    events
        .goals()
        .isShootingTrigger()
        .and(events.turret().isInToleranceTrigger())
        .whileTrue(indexer.indexingCommand())
        .whileFalse(indexer.idleCommand());

    // Power management
    events
        .power()
        .isNormalProfileTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  indexer.setIndexerSupplyCurrentLimit(22.0);
                  indexer.setFeederSupplyCurrentLimit(40.0);
                }));
    events
        .power()
        .isShootingProfileTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  indexer.setIndexerSupplyCurrentLimit(22.0);
                  indexer.setFeederSupplyCurrentLimit(40.0);
                }));
    events
        .power()
        .isLowVoltageProfileTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  indexer.setIndexerSupplyCurrentLimit(15.0);
                  indexer.setFeederSupplyCurrentLimit(25.0);
                }));
  }
}
