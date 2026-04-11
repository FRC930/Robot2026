package frc.robot.subsystems.extender;

import frc.robot.util.AllEvents;
import frc.robot.util.SubsystemBehavior;

public class ExtenderBehavior extends SubsystemBehavior {
  private final ExtenderSubsystem extender;

  public ExtenderBehavior(ExtenderSubsystem extender) {
    this.extender = extender;
  }

  @Override
  public void configure(AllEvents events) {
    events.goals().isIdleTrigger().whileTrue(this.extender.idleCommand());
    events.goals().isOuttakingTrigger().whileTrue(this.extender.outtakeCommand());
    events.goals().isIntakingTrigger().whileTrue(this.extender.intakeCommand());
    events
        .goals()
        .isShootingTrigger()
        .and(events.drive().isNotMoving().negate())
        .whileTrue(this.extender.shootingCommand());
    events
        .goals()
        .isShootingTrigger()
        .and(events.drive().isNotMoving())
        .whileTrue(this.extender.agitateCommand());
    events.goals().isRaisedIntakeTrigger().whileTrue(this.extender.raisedCommand());
  }
}
