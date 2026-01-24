package frc.robot.subsystems.intake;

import frc.robot.goals.RobotGoalEvents;
import frc.robot.state.MatchStateEvents;
import frc.robot.subsystems.climber.ClimberEvents;
import frc.robot.util.SubsystemBehavior;

public class IntakeBehavior extends SubsystemBehavior {

  private final IntakeSubsystem intake;

  public IntakeBehavior(IntakeSubsystem intake) {
    this.intake = intake;
  }

  @Override
  public void configure(
      RobotGoalEvents goals,
      MatchStateEvents matchState,
      IntakeEvents intakeState,
      ClimberEvents climberState) {
    goals.isLaunchingTrigger().whileTrue(this.intake.intakeCommand());
    goals.isIdleTrigger().whileTrue(this.intake.idleCommand());
    // goals.isClimbing().whileTrue(this.intake.idleCommand());
  }
}
