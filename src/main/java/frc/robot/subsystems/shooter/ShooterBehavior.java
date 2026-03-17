package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
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

    // Power management
    events
        .power()
        .isNormalProfileTrigger()
        .onTrue(Commands.runOnce(() -> shooter.setSupplyCurrentLimit(40.0)));
    events
        .power()
        .isShootingProfileTrigger()
        .onTrue(Commands.runOnce(() -> shooter.setSupplyCurrentLimit(40.0)));
    events
        .power()
        .isLowVoltageProfileTrigger()
        .onTrue(Commands.runOnce(() -> shooter.setSupplyCurrentLimit(35.0)));
  }
}
