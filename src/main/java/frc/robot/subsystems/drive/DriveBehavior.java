package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.AllEvents;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SubsystemBehavior;

public class DriveBehavior extends SubsystemBehavior {
  private static final double NO_SPEED_LIMIT = 1.0;

  private final LoggedTunableNumber intakingSpeedLimit =
      new LoggedTunableNumber("Drive/IntakingSpeedLimit", 0.6);
  private final LoggedTunableNumber shootingSpeedLimit =
      new LoggedTunableNumber("Drive/ShootingSpeedLimit", 0.5);

  private final Drive drive;

  public DriveBehavior(Drive drive) {
    this.drive = drive;
  }

  @Override
  public void configure(AllEvents events) {
    events
        .goals()
        .isIntakingTrigger()
        .whileTrue(
            Commands.startEnd(
                () -> drive.setGoalSpeedLimit(intakingSpeedLimit.get()),
                () -> drive.setGoalSpeedLimit(NO_SPEED_LIMIT)));

    events
        .goals()
        .isShootingTrigger()
        .whileTrue(
            Commands.startEnd(
                () -> drive.setGoalSpeedLimit(shootingSpeedLimit.get()),
                () -> drive.setGoalSpeedLimit(NO_SPEED_LIMIT)));

    // Power management: throttle drive supply current based on active power profile
    events
        .power()
        .isNormalProfileTrigger()
        .onTrue(Commands.runOnce(() -> drive.setDriveSupplyCurrentLimit(60.0)));
    events
        .power()
        .isShootingProfileTrigger()
        .onTrue(Commands.runOnce(() -> drive.setDriveSupplyCurrentLimit(40.0)));
    events
        .power()
        .isLowVoltageProfileTrigger()
        .onTrue(Commands.runOnce(() -> drive.setDriveSupplyCurrentLimit(30.0)));
  }
}
