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
    events
        .goals()
        .isShootingTrigger()
        .whileTrue(shooter.shooterCommand())
        .whileFalse(shooter.idleCommand());
  }
}
