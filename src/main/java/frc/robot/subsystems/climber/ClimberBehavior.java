package frc.robot.subsystems.climber;

import frc.robot.goals.RobotGoalEvents;
import frc.robot.state.MatchStateEvents;
import frc.robot.subsystems.intake.IntakeEvents;
import frc.robot.util.SubsystemBehavior;

public class ClimberBehavior extends SubsystemBehavior {

  private final ClimberSubsystem climber;

  public ClimberBehavior(ClimberSubsystem climber) {
    this.climber = climber;
  }

  @Override
  public void configure(
      RobotGoalEvents goals,
      MatchStateEvents matchState,
      IntakeEvents intakeState,
      ClimberEvents climberState) {
    // goals.isClimbing().whileTrue(climber.goToL0Command());
    // goals.isClimbing().whileTrue(climber.goToL1Command());
    // goals.isClimbing().whileTrue(climber.goToL2Command());
    // goals.isClimbing().whileTrue(climber.goToL3Command());
  }
}
