package frc.robot.state;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface MatchEvents {

  public Trigger isDisabled();

  public Trigger isAutonomousEnable();

  public Trigger isEnabled();

  public Trigger isTeleopEnabled();
}
