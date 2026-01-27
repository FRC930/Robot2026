package frc.robot.goals;

import frc.robot.state.MatchStateEvents;
import frc.robot.subsystems.climber.ClimberEvents;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeEvents;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterEvents;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.SubsystemBehavior;

public class RobotSubsystemBehavior extends SubsystemBehavior {

  private final IntakeSubsystem intake;
  private final ShooterSubsystem shooter;
  private final ClimberSubsystem climber;

  public RobotSubsystemBehavior(
      IntakeSubsystem intake, ShooterSubsystem shooter, ClimberSubsystem climber) {
    this.intake = intake;
    this.shooter = shooter;
    this.climber = climber;
  }

  @Override
  public void configure(
      RobotGoalEvents goals,
      MatchStateEvents matchState,
      IntakeEvents intakeEvents,
      ClimberEvents climberEvents,
      ShooterEvents shooterEvents) {
    // intake behavior
    goals.isLaunchingTrigger().whileTrue(this.intake.intakeCommand());
    goals.isIdleTrigger().whileTrue(this.intake.idleCommand());
    goals.isIntakingTrigger().whileTrue(this.intake.intakeCommand());
    goals.isOuttakingTrigger().whileTrue(this.intake.outtakeCommand());
    goals.isL1ClimbingTrigger().whileTrue(this.intake.idleCommand());
    goals.isL2ClimbingTrigger().whileTrue(this.intake.idleCommand());
    goals.isL3ClimbingTrigger().whileTrue(this.intake.idleCommand());

    // climber behavior
    goals.isLaunchingTrigger().whileTrue(this.climber.goToIdleCommand());
    goals.isIdleTrigger().whileTrue(this.climber.goToIdleCommand());
    goals.isIntakingTrigger().whileTrue(this.climber.goToIdleCommand());
    goals.isOuttakingTrigger().whileTrue(this.climber.goToIdleCommand());
    goals.isL1ClimbingTrigger().whileTrue(this.climber.goToL1Command());
    goals.isL2ClimbingTrigger().whileTrue(this.climber.goToL2Command());
    goals.isL3ClimbingTrigger().whileTrue(this.climber.goToL3Command());

    // shooter behavior
    goals.isLaunchingTrigger().whileTrue(this.shooter.shooterCommand());
    goals.isIdleTrigger().whileTrue(this.shooter.idleCommand());
    goals.isIntakingTrigger().whileTrue(this.shooter.shooterCommand());
    goals.isOuttakingTrigger().whileTrue(this.shooter.idleCommand());
    goals.isL1ClimbingTrigger().whileTrue(this.shooter.idleCommand());
    goals.isL2ClimbingTrigger().whileTrue(this.shooter.idleCommand());
    goals.isL3ClimbingTrigger().whileTrue(this.shooter.idleCommand());
  }
}
