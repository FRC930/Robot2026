package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface FeederEvents {
  public Trigger isIdleTrigger();

  public Trigger isFeedingTrigger();
}
