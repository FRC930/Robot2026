package frc.robot.subsystems.extender;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ExtenderEvents {
  public Trigger idleTrigger();

  public Trigger intakingTrigger();

  public Trigger outtakingTrigger();

  public Trigger shootingTrigger();

  public Trigger raisedTrigger();

  public Trigger agitatingTrigger();
}
