package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.AllEvents;
import frc.robot.util.SubsystemBehavior;

public class IntakeBehavior extends SubsystemBehavior {

  private final IntakeSubsystem intake;

  public IntakeBehavior(IntakeSubsystem intake) {
    this.intake = intake;
  }

  // TODO may have to wait for turrent to be out of position and may have to be in behavior.
  @Override
  public void configure(AllEvents events) {
    events.goals().isIdleTrigger().whileTrue(this.intake.idleCommand());
    events.goals().isOuttakingTrigger().whileTrue(this.intake.outtakeCommand());
    events.goals().isIntakingTrigger().whileTrue(this.intake.intakeCommand());
    events
        .goals()
        .isShootingTrigger()
        .and(events.drive().isNotMoving().negate())
        .whileTrue(this.intake.shootingCommand());
    events
        .goals()
        .isShootingTrigger()
        .and(events.drive().isNotMoving())
        .whileTrue(this.intake.agitateCommand());
    events.goals().isRaisedIntakeTrigger().whileTrue(this.intake.raisedCommand());
    events.goals().isClimbingL0().whileTrue(this.intake.idleCommand());
    events.goals().isClimbingL1().whileTrue(this.intake.idleCommand());
    events.goals().isClimbingL2().whileTrue(this.intake.idleCommand());
    events.goals().isClimbingL3().whileTrue(this.intake.idleCommand());

    // Power management
    events
        .power()
        .isNormalProfileTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  intake.setRollerSupplyCurrentLimit(40.0);
                  intake.setExtenderSupplyCurrentLimit(80.0);
                }));
    events
        .power()
        .isShootingProfileTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  intake.setRollerSupplyCurrentLimit(30.0);
                  intake.setExtenderSupplyCurrentLimit(60.0);
                }));
    events
        .power()
        .isLowVoltageProfileTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  intake.setRollerSupplyCurrentLimit(20.0);
                  intake.setExtenderSupplyCurrentLimit(40.0);
                }));
  }
}
