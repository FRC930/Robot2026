package frc.robot.subsystems.shooter;

import frc.robot.util.AllEvents;
import frc.robot.util.SubsystemBehavior;

public class ShooterBehavior extends SubsystemBehavior {

  private final ShooterSubsystem shooter;

  public ShooterBehavior(ShooterSubsystem shooter) {
    this.shooter = shooter;
  }

  @Override
  public void configure(AllEvents events) {
    // Pre-spin shooter when AIMING so it's ready to fire instantly
    events.goals().isAimingTrigger().whileTrue(shooter.prespinCommand());
    events
        .goals()
        .isShootingTrigger()
        .or(events.goals().isPassingTrigger())
        .whileTrue(shooter.shooterCommand())
        .whileFalse(shooter.idleCommand());
  }
}
