package frc.robot.subsystems.compactor;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface CompactorEvents {
  public Trigger idleTrigger();

  public Trigger goToBottomTrigger();

  public Trigger goToTopTrigger();
}
