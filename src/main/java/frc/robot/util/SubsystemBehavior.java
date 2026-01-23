package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

/**
 * Base class for hardware subsystem behaviors that react to robot goals. These behaviors translate
 * robot goals into hardware subsystem state changes.
 *
 * <p>Subsystem behaviors listen to the API layer (RobotGoalEvents) and drive the hardware layer
 * (Launcher, Drive, etc.).
 *
 * <p>Usage:
 *
 * <pre>{@code
 * public class LauncherBehavior extends SubsystemBehavior {
 *     private final LauncherSubsystem launcher;
 *
 *     public LauncherBehavior(LauncherSubsystem launcher) {
 *         super();
 *         this.launcher = launcher;
 *     }
 *
 *     @Override
 *     public void configure(RobotGoalEvents goals, MatchStateEvents matchState) {
 *         goals.isScoring().whileTrue(launcher.runLauncher());
 *     }
 * }
 * }</pre>
 *
 * <p>TODO (students): Add additional subsystem event interfaces to the signature as you create them
 * (e.g., LauncherEvents, DriveEvents)
 */
public abstract class SubsystemBehavior extends Behavior<SubsystemBehavior> {

  private static final List<SubsystemBehavior> subsystemBehaviors = new ArrayList<>();

  protected SubsystemBehavior() {
    super();
    subsystemBehaviors.add(this);
  }

  /**
   * Configure all registered subsystem behaviors.
   *
   * @param goals Robot goal events to react to
   * @param matchState Match phase events (disabled, auto, teleop)
   * @param launcher
   */
  public static final void configureAll(AllEvents events) {
    for (SubsystemBehavior behavior : subsystemBehaviors) {
      behavior.configure(events);
    }
  }

  /**
   * Configure this subsystem behavior's trigger bindings.
   *
   * @param goals Robot goal events to react to
   * @param matchState Match phase events (disabled, auto, teleop)
   * @param launcher
   */
  public abstract void configure(AllEvents events);
}
