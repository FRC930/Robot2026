package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A helper interface explaining how to implement a state-based Subsystem.
 *
 * @param <T> The SubsystemState enum type we check states for.
 */
public interface StateSubsystem<T> {
  /**
   * Returns the trigger for when a subsystem is in a desired state.
   *
   * @param state The desired state to check for.
   * @return Trigger for when the Subsystem's state is {@code state}.
   */
  public Trigger isStateTrigger(T state);
}
