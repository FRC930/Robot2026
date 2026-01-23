package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * A helper interface explaining how to implement a state-based Subsystem.
 *
 * @param <T> The SubsystemState enum type we set states for.
 */
public interface MutStateSubsystem<T> {
  /**
   * Returns the command to set a subsystem to a desired state.
   *
   * @param state The desired state to switch to.
   * @return Command to set the Subsystem's state to {@code state}.
   */
  public Command setStateCommand(T state);
}
