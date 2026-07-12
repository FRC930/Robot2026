package frc.robot.goals;

import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.operator.OperatorIntentEvents;
import frc.robot.util.GoalBehavior;

/**
 * Wires operator button presses to robot goal state.
 *
 * <p>This is the teleop-specific logic. Autonomous bypasses this and calls RobotGoals.setGoal()
 * directly.
 *
 * <p>TODO (students): Map your button intents to goals here Example:
 * intent.wantsToScore().onTrue(goals.setGoal(RobotGoal.LAUNCHING))
 * .onFalse(goals.setGoal(RobotGoal.IDLE));
 */
public class RobotGoalsBehavior extends GoalBehavior {

  private RobotGoals goals;

  public RobotGoalsBehavior(RobotGoals goals) {
    this.goals = goals;
  }

  @Override
  public void configure(OperatorIntentEvents intent) {
    intent
        .wantsToOuttake()
        .onTrue(goals.setGoalCommand(RobotGoal.OUTTAKING))
        .onFalse(goals.setGoalCommand(RobotGoal.IDLE));

    intent
        .wantsToIntake()
        .onTrue(goals.setGoalCommand(RobotGoal.INTAKING))
        .onFalse(goals.setGoalCommand(RobotGoal.IDLE));

    intent
        .wantsToScoreTrigger()
        .whileTrue(new RepeatCommand(goals.setGoalCommand(RobotGoal.SHOOTING)))
        .whileFalse(goals.setGoalCommand(RobotGoal.IDLE));

    intent
        .wantsToRaiseIntake()
        .whileTrue(goals.setGoalCommand(RobotGoal.RAISED_INTAKE))
        .whileFalse(goals.setGoalCommand(RobotGoal.IDLE));

    intent
        .wantsToPrespin()
        .and(intent.wantsToScoreTrigger().negate())
        .whileTrue(goals.setGoalCommand(RobotGoal.PRESPIN))
        .whileFalse(goals.setGoalCommand(RobotGoal.IDLE));
  }
}
