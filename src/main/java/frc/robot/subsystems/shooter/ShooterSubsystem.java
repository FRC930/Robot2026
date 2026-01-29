package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase implements ShooterEvents {
  private ShooterIO m_IO;
  private LoggedTunableNumber setpoint = new LoggedTunableNumber("Shooter/setpoint", 60.0);

  private final EnumState<ShooterState> currentGoal =
      new EnumState<>("Shooter/States", ShooterState.IDLE);

  private ShooterInputsAutoLogged logged = new ShooterInputsAutoLogged();

  public LoggedTunableGainsBuilder tunableGains =
      new LoggedTunableGainsBuilder(
          "Gains/ShooterSubsystem/", 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  public ShooterSubsystem(ShooterIO IO) {
    m_IO = IO;
    logged.shooterAngularVelocity = DegreesPerSecond.mutable(0);
    logged.shooterSetpoint = RPM.mutable(0);
    this.m_IO.setGains(tunableGains.build());
  }

  /**
   * Sets the speed for the shooter
   *
   * @param speed
   */
  public void setShooterSpeed(AngularVelocity speed) {
    m_IO.setShooterTarget(speed);
  }

  public Command shooterCommand() {
    return runOnce(
        () -> {
          currentGoal.set(ShooterState.SHOOTING);
        });
  }

  public Command idleCommand() {
    return runOnce(
        () -> {
          currentGoal.set(ShooterState.IDLE);
        });
  }

  public void stop() {
    m_IO.stop();
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/Shooter", logged);
    switch (currentGoal.get()) {
      case SHOOTING:
        setShooterSpeed(RPM.of(setpoint.get()));
        break;
      case IDLE:
        stop();
        break;
    }
    tunableGains.ifGainsHaveChanged((gains) -> this.m_IO.setGains(gains));
  }

  @Override
  public Trigger isIdleTrigger() {
    return currentGoal.is(ShooterState.IDLE);
  }

  @Override
  public Trigger isShootingTrigger() {
    return currentGoal.is(ShooterState.SHOOTING);
  }
}
