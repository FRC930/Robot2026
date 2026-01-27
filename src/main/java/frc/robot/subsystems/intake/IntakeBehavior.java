package frc.robot.subsystems.intake;

import frc.robot.util.AllEvents;
import frc.robot.util.SubsystemBehavior;

public class IntakeBehavior extends SubsystemBehavior {

  private final IntakeSubsystem intake;

  public IntakeBehavior(IntakeSubsystem intake) {
    this.intake = intake;
  }

  @Override
  public void configure(AllEvents events) {
    events.goals().isShootingTrigger().whileTrue(this.intake.intakeCommand());
    events.goals().isIdleTrigger().whileTrue(this.intake.idleCommand());
  }
}
