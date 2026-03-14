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
  }
}
