package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase implements ShooterEvents {

  private ShooterIO m_IO;

  private final EnumState<ShooterState> currentGoal =
      new EnumState<>("Shooter/States", ShooterState.IDLE);

  private ShooterInputsAutoLogged logged = new ShooterInputsAutoLogged();

  public ShooterSubsystem(ShooterIO IO) {
    m_IO = IO;
    logged.shooterAngularVelocity = DegreesPerSecond.mutable(0);
    logged.shooterVoltage = Volts.mutable(0);
    logged.shooterSetVoltage = Volts.mutable(0);
  }

  /**
   * Sets the speed for the shooter
   *
   * @param speed
   */
  public void setShooterSpeed(Voltage speed) {
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
        setShooterSpeed(Volts.of(11.0));
        break;
      case IDLE:
        stop();
        break;
    }
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
