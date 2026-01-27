package frc.robot.goals;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface RobotEvents {

  public Trigger isIdleTrigger();

  public Trigger isIntakingTrigger();

  public Trigger isOuttakingTrigger();

  public Trigger isShootingTrigger();
}
