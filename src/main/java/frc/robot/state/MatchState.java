package frc.robot.state;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.StateSubsystem;
import frc.robot.util.VirtualSubsystem;

/**
 * Exposes match phase state as triggers for behaviors to react to. Wraps DriverStation calls into a
 * reactive interface.
 */
public class MatchState extends VirtualSubsystem implements StateSubsystem<MatchStates> {

  @Override
  public void periodic() {
    // No periodic updates needed - triggers poll DriverStation directly
  }

  @Override
  public Trigger isStateTrigger(MatchStates state) {

    switch (state) {
      case DISABLED:
        return new Trigger(DriverStation::isDisabled);
      case AUTONOMOUS:
        return new Trigger(DriverStation::isAutonomousEnabled);
      case ENABLED:
        return new Trigger(DriverStation::isEnabled);
      case TELEOP:
        return new Trigger(DriverStation::isTeleopEnabled);
    }

    // SHOULD NOT EVER GET HERE
    throw new UnsupportedOperationException("Undefined state trigger for " + state.toString());
  }
}
