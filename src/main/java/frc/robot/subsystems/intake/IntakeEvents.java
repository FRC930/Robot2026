package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IntakeEvents {
  public Trigger isIdleTrigger();

  public Trigger isIntakingTrigger();

  public Trigger isOuttakingTrigger();

  public Trigger isShootingTrigger();

  public Trigger isRaisedTrigger();
}
