package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.power.PowerProfile;
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
                  intake.setRollerSupplyCurrentLimit(PowerProfile.NORMAL.intakeRollerSupplyLimit);
                  intake.setExtenderSupplyCurrentLimit(
                      PowerProfile.NORMAL.intakeExtenderSupplyLimit);
                }));
    events
        .power()
        .isShootingProfileTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  intake.setRollerSupplyCurrentLimit(PowerProfile.SHOOTING.intakeRollerSupplyLimit);
                  intake.setExtenderSupplyCurrentLimit(
                      PowerProfile.SHOOTING.intakeExtenderSupplyLimit);
                }));
    events
        .power()
        .isLowVoltageProfileTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  intake.setRollerSupplyCurrentLimit(
                      PowerProfile.LOW_VOLTAGE.intakeRollerSupplyLimit);
                  intake.setExtenderSupplyCurrentLimit(
                      PowerProfile.LOW_VOLTAGE.intakeExtenderSupplyLimit);
                }));
  }
}
