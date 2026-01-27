package frc.robot.state;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.VirtualSubsystem;

/**
 * Exposes match phase state as triggers for behaviors to react to. Wraps DriverStation calls into a
 * reactive interface.
 */
public class MatchState extends VirtualSubsystem implements MatchEvents {

  @Override
  public void periodic() {
    // No periodic updates needed - triggers poll DriverStation directly
  }

  @Override
  public Trigger isDisabled() {
    return new Trigger(DriverStation::isDisabled);
  }

  @Override
  public Trigger isAutonomousEnable() {
    return new Trigger(DriverStation::isAutonomousEnabled);
  }

  @Override
  public Trigger isEnabled() {
    return new Trigger(DriverStation::isEnabled);
  }

  @Override
  public Trigger isTeleopEnabled() {
    return new Trigger(DriverStation::isTeleopEnabled);
  }
}
