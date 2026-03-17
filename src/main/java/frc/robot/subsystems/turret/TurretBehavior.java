package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.power.PowerProfile;
import frc.robot.util.AllEvents;
import frc.robot.util.SubsystemBehavior;

// TODO Later
public class TurretBehavior extends SubsystemBehavior {

  private final TurretSubsystem turret;

  public TurretBehavior(TurretSubsystem turret) {
    this.turret = turret;
  }

  @Override
  public void configure(AllEvents events) {
    // Power management
    events
        .power()
        .isNormalProfileTrigger()
        .onTrue(
            Commands.runOnce(
                () -> turret.setSupplyCurrentLimit(PowerProfile.NORMAL.turretSupplyLimit)));
    events
        .power()
        .isShootingProfileTrigger()
        .onTrue(
            Commands.runOnce(
                () -> turret.setSupplyCurrentLimit(PowerProfile.SHOOTING.turretSupplyLimit)));
    events
        .power()
        .isLowVoltageProfileTrigger()
        .onTrue(
            Commands.runOnce(
                () -> turret.setSupplyCurrentLimit(PowerProfile.LOW_VOLTAGE.turretSupplyLimit)));
  }
}
