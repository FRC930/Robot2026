package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IndexerEvents {
  public Trigger isIdleTrigger();

  public Trigger isIndexingTrigger();
}
