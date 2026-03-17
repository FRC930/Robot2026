package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.power.PowerProfile;
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
        .onTrue(
            Commands.runOnce(
                () -> shooter.setSupplyCurrentLimit(PowerProfile.NORMAL.shooterSupplyLimit)));
    events
        .power()
        .isShootingProfileTrigger()
        .onTrue(
            Commands.runOnce(
                () -> shooter.setSupplyCurrentLimit(PowerProfile.SHOOTING.shooterSupplyLimit)));
    events
        .power()
        .isLowVoltageProfileTrigger()
        .onTrue(
            Commands.runOnce(
                () -> shooter.setSupplyCurrentLimit(PowerProfile.LOW_VOLTAGE.shooterSupplyLimit)));
  }
}
