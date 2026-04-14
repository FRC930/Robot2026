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
        .wantsToOuttake()
        .onTrue(goals.setGoalCommand(RobotGoal.OUTTAKING))
        .onFalse(goals.setGoalCommand(RobotGoal.IDLE));

    intent
        .wantsToIntake()
        .onTrue(goals.setGoalCommand(RobotGoal.INTAKING))
        .onFalse(goals.setGoalCommand(RobotGoal.IDLE));

    intent
        .wantsToScoreTrigger()
        .and(intent.wantsToRevIndexer().negate())
        .whileTrue(goals.setGoalCommand(RobotGoal.SHOOTING));

    intent.wantsToRevIndexer().whileTrue(goals.setGoalCommand(RobotGoal.REVERSE_INDEXER));

    intent
        .wantsToRaiseIntake()
        .whileTrue(goals.setGoalCommand(RobotGoal.RAISED_INTAKE))
        .whileFalse(goals.setGoalCommand(RobotGoal.IDLE));

    intent
        .wantsToScoreTrigger()
        .or(intent.wantsToRevIndexer())
        .or(intent.wantsToPass())
        .negate()
        .whileTrue(goals.setGoalCommand(RobotGoal.IDLE));
    intent
        .wantsToRaiseCompactor()
        .whileTrue(goals.setGoalCommand(RobotGoal.RAISING_COMPACTOR))
        .onFalse(goals.setGoalCommand(RobotGoal.IDLE));
    intent
        .wantsToLowerCompactor()
        .whileTrue(goals.setGoalCommand(RobotGoal.LOWERING_COMPACTOR))
        .onFalse(goals.setGoalCommand(RobotGoal.IDLE));
  }
}
