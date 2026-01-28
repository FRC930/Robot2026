package frc.robot.goals;

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
        .wantsToIntakeTrigger()
        .onTrue(goals.setGoalCommand(RobotGoal.INTAKING))
        .onFalse(goals.setGoalCommand(RobotGoal.IDLE));
    intent
        .wantsToScoreTrigger()
        .onTrue(goals.setGoalCommand(RobotGoal.SHOOTING))
        .onFalse(goals.setGoalCommand(RobotGoal.IDLE));
    intent
        .wantsToOutake()
        .onTrue(goals.setGoalCommand(RobotGoal.OUTTAKING))
        .onFalse(goals.setGoalCommand(RobotGoal.IDLE));

    intent
        .wantsToClimbL0()
        .onTrue(goals.setGoalCommand(RobotGoal.CLIMBINGL0))
        .onFalse(goals.setGoalCommand(RobotGoal.IDLE));
    intent
        .wantsToClimbL1()
        .onTrue(goals.setGoalCommand(RobotGoal.CLIMBINGL1))
        .onFalse(goals.setGoalCommand(RobotGoal.IDLE));

    intent
        .wantsToClimbL2()
        .onTrue(goals.setGoalCommand(RobotGoal.CLIMBINGL2))
        .onFalse(goals.setGoalCommand(RobotGoal.IDLE));

    intent
        .wantsToClimbL3()
        .onTrue(goals.setGoalCommand(RobotGoal.CLIMBINGL3))
        .onFalse(goals.setGoalCommand(RobotGoal.IDLE));
  }
}
