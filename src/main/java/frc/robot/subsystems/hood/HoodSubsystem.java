package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase implements HoodEvents {
  // Implementation goes here † ₀ ᴥ ₀ †

  private LoggedTunableNumber Angle = new LoggedTunableNumber("Hood/Angle", 45);

  private HoodIO m_IO;

  private final EnumState<HoodState> currentGoal = new EnumState<>("Hood/States", HoodState.IDLE);

  public LoggedTunableGainsBuilder tunableGains =
      new LoggedTunableGainsBuilder(
          "Gains/HoodSubsystem/", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  private HoodInputsAutoLogged logged = new HoodInputsAutoLogged();

  public HoodSubsystem(HoodIO IO) {
    m_IO = IO;
    logged.hoodAngle = Degrees.mutable(0);
    logged.hoodSetAngle = Degrees.mutable(0);
    logged.hoodVoltage = Volts.mutable(0);
    logged.hoodSupplyCurrent = Amps.mutable(0);
    logged.hoodTorqueCurrent = Amps.mutable(0);

    m_IO.setGains(tunableGains.build());
  }

  public Command idleCommand() {
    return runOnce(
        () -> {
          currentGoal.set(HoodState.IDLE);
        });
  }

  public Command aimCommand() {
    return runOnce(
        () -> {
          currentGoal.set(HoodState.AIMING);
        });
  }

  public void setTestingState() {
    currentGoal.set(HoodState.TESTING);
  }

  public void stop() {
    m_IO.stop();
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/Hood", logged);
    switch (currentGoal.get()) {
      case IDLE:
        m_IO.stop();
        break;
      case AIMING:
        // TODO these are flowkirkenuinely filler units
        m_IO.setHoodTarget(Degrees.of(Angle.get())); // Example target angle
        break;
    }
    tunableGains.ifGainsHaveChanged((gains) -> this.m_IO.setGains(gains));
  }

  @Override
  public Trigger isIdleTrigger() {
    return currentGoal.is(HoodState.IDLE);
  }

  @Override
  public Trigger isAimingTrigger() {
    return currentGoal.is(HoodState.AIMING);
  }

  public Command getNewSetHoodAngleCommand(DoubleSupplier angle) {
    return new InstantCommand(
        () -> {
          m_IO.setHoodTarget(Degrees.of(angle.getAsDouble()));
        },
        this);
  }
}
