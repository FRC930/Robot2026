package frc.robot.util;

import frc.robot.goals.RobotGoalEvents;
import frc.robot.state.MatchStateEvents;
import frc.robot.subsystems.climber.ClimberEvents;
import frc.robot.subsystems.intake.IntakeEvents;
import java.util.ArrayList;
import java.util.List;

/**
 * Base class for hardware subsystem behaviors that react to robot goals. These behaviors translate
 * robot goals into hardware subsystem state changes.
 *
 * <p>Subsystem behaviors listen to the API layer (RobotGoalEvents) and drive the hardware layer
 * (intake, Drive, etc.).
 *
 * <p>Usage:
 *
 * <pre>{@code
 * public class intakeBehavior extends SubsystemBehavior {
 *     private final intakeSubsystem intake;
 *
 *     public intakeBehavior(intakeSubsystem intake) {
 *         super();
 *         this.intake = intake;
 *     }
 *
 *     @Override
 *     public void configure(RobotGoalEvents goals, MatchStateEvents matchState) {
 *         goals.isScoring().whileTrue(intake.runintake());
 *     }
 * }
 * }</pre>
 *
 * <p>TODO (students): Add additional subsystem event interfaces to the signature as you create them
 * (e.g., intakeEvents, DriveEvents)
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
   * @param intake
   */
  public static void configureAll(
      RobotGoalEvents goals,
      MatchStateEvents matchState,
      IntakeEvents intake,
      ClimberEvents climber) {
    for (SubsystemBehavior behavior : subsystemBehaviors) {
      behavior.configure(goals, matchState, intake, climber);
    }
  }

  // /**
  //  * Configure this subsystem behavior's trigger bindings.
  //  *
  //  * @param goals Robot goal events to react to
  //  * @param matchState Match phase events (disabled, auto, teleop)
  //  * @param intake
  //  */
  // public abstract void configure(
  //     RobotGoalEvents goals, MatchStateEvents matchState, IntakeEvents intake);

  /**
   * Configure this subsystem behavior's trigger bindings.
   *
   * @param goals Robot goal events to react to
   * @param matchState Match phase events (disabled, auto, teleop)
   * @param intake
   */
  public abstract void configure(
      RobotGoalEvents goals,
      MatchStateEvents matchState,
      IntakeEvents intake,
      ClimberEvents climber);
}
