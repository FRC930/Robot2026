// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Sets the controless the intake and endexer */
public class IntakeSubsystem extends SubsystemBase implements IntakeEvents {
  /** Creates a new ExampleSubsystem. */
  private IntakeIO m_IO;

  private double rpm =
      (11.0 / 12.0) * IntakeIOTalonFX.KRACKEN_X60_FOC_MAX_RPM.in(RPM) / IntakeIOTalonFX.GEAR_RATIO;

  private LoggedTunableNumber intakeTargetRPM =
      new LoggedTunableNumber("Intake/intakeTargetRPM", rpm);
  private LoggedTunableNumber intakeExtenderTargetVolts =
      new LoggedTunableNumber("Intake/intakeExtenderTargetVolts", 5);
  private final EnumState<IntakeState> currentGoal =
      new EnumState<>("Intake/States", IntakeState.IDLE);

  public LoggedTunableGainsBuilder tunableGains =
      new LoggedTunableGainsBuilder(
          "Gains/IntakeSubsystem/", 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  private IntakeInputsAutoLogged logged = new IntakeInputsAutoLogged();

  public IntakeSubsystem(IntakeIO IO) {
    m_IO = IO;
    logged.intakeAngularVelocity = RPM.mutable(0);
    logged.intakeSetAngularVelocity = RPM.mutable(0);
    logged.intakeSupplyCurrent = Amps.mutable(0);
    logged.intakeExtenderVoltage = Volts.mutable(0);
    logged.intakeExtenderSetVoltage = Volts.mutable(0);
    logged.intakeExtenderSupplyCurrent = Amps.mutable(0);
    logged.intakeExtenderAngle = Radians.mutable(IntakeIOSim.kMinvoltageRads);
  }

  /**
   * Sets the speed for the intake
   *
   * @param speed
   */
  public void setIntakeSpeed(AngularVelocity speed) {
    m_IO.setIntakeTarget(speed);
  }

  /**
   * Sets the speed for the intake
   *
   * @param speed
   */
  public void setIntakeVoltage(Voltage voltage) {
    m_IO.setIntakeExtenderTarget(voltage);
  }

  public Command intakeCommand() {
    return runOnce(
        () -> {
          currentGoal.set(IntakeState.INTAKING);
        });
  }

  public Command outtakeCommand() {
    return runOnce(
        () -> {
          currentGoal.set(IntakeState.OUTTAKING);
        });
  }

  public Command idleCommand() {
    return runOnce(
        () -> {
          currentGoal.set(IntakeState.IDLE);
        });
  }

  public void setTestingState() {
    currentGoal.set(IntakeState.TESTING);
  }

  public void stop() {
    m_IO.stop();
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/Intake", logged);
    switch (currentGoal.get()) {
      case INTAKING:
        // TODO filler units rn
        m_IO.setIntakeTarget(RPM.of(intakeTargetRPM.get()));
        m_IO.setIntakeExtenderTarget(Volts.of(intakeExtenderTargetVolts.get()));
        break;
      case OUTTAKING:
        // TODO filler units rn
        m_IO.setIntakeTarget(RPM.of(-intakeTargetRPM.get()));
        m_IO.setIntakeExtenderTarget(Volts.of(intakeExtenderTargetVolts.get()));
        break;
      case IDLE:
        stop();
        break;
    }
    tunableGains.ifGainsHaveChanged((gains) -> this.m_IO.setGains(gains));
  }

  @Override
  public Trigger isIdleTrigger() {
    return currentGoal.is(IntakeState.IDLE);
  }

  @Override
  public Trigger isIntakingTrigger() {
    return currentGoal.is(IntakeState.INTAKING);
  }

  @Override
  public Trigger isOuttakingTrigger() {
    return currentGoal.is(IntakeState.OUTTAKING);
  }

  public Command getNewSetIntakeVelocityCommand(DoubleSupplier rpm) {
    return new InstantCommand(
        () -> {
          m_IO.setIntakeTarget(RPM.of(rpm.getAsDouble()));
        },
        this);
  }

  public Command getNewSetIntakeExtenderVoltsCommand(DoubleSupplier volts, boolean negate) {
    return new InstantCommand(
        () -> {
          m_IO.setIntakeExtenderTarget(
              Volts.of((negate) ? -1 * volts.getAsDouble() : volts.getAsDouble()));
        },
        this);
  }
}
