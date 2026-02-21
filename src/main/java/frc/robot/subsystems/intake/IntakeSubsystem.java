// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotVisualization;
import frc.robot.util.EnumState;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Sets the controless the intake and endexer */
public class IntakeSubsystem extends SubsystemBase implements IntakeEvents {
  /** Creates a new ExampleSubsystem. */
  private IntakeIO m_IO;

  private double rpm =
      (11.0 / 12.0)
          * IntakeIOTalonFX.KRACKEN_X60_FOC_MAX_RPM.in(RPM)
          / IntakeIOTalonFX.GEAR_RATIO_ROLLERS;

  private LoggedTunableNumber intakeTargetRPM =
      new LoggedTunableNumber("Intake/intakeTargetRPM", rpm);
  private LoggedTunableNumber intakeExtenderTargetAngleUp =
      new LoggedTunableNumber("Intake/intakeExtenderTargetAngle", 90.0);
  private LoggedTunableNumber intakeExtenderTargetAngleDown =
      new LoggedTunableNumber("Intake/intakeExtenderTargetAngle", 0.0);
  private final EnumState<IntakeState> currentGoal =
      new EnumState<>("Intake/States", IntakeState.IDLE);

  public LoggedTunableGainsBuilder rollerGains =
      new LoggedTunableGainsBuilder(
          "Gains/IntakeSubsystem/", 0.4, 0, 0.02, 0.33, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  public LoggedTunableGainsBuilder extenderGains =
      new LoggedTunableGainsBuilder(
          "Gains/ExtenderSubsystem/", 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  private IntakeInputsAutoLogged logged = new IntakeInputsAutoLogged();

  public IntakeSubsystem(IntakeIO IO) {
    m_IO = IO;
    logged.rollerVelocity = RPM.mutable(0.0);
    logged.rollerVelocitySetPoint = RPM.mutable(0.0);
    logged.rollerSupplyCurrent = Amps.mutable(0.0);
    logged.rollerTorqueCurrent = Amps.mutable(0);
    logged.rollerVoltage = Volts.mutable(0);
    logged.leaderRollerTemp = Fahrenheit.mutable(0);
    logged.followerRollerTemp = Fahrenheit.mutable(0);
    logged.extenderVoltage = Volts.mutable(0.0);
    logged.extenderAngle = Degrees.mutable(0.0);
    logged.extenderAngleSetPoint = Degrees.mutable(0.0);
    logged.extenderSupplyCurrent = Amps.mutable(0.0);
    logged.extenderTorqueCurrent = Amps.mutable(0);
    // logged.extenderEmulatedAngle = Radians.mutable(0);
    // logged.extenderEmulatedSetAngle = Radians.mutable(0);
    m_IO.setRollerGains(rollerGains.build());
    m_IO.setExtenderGains(extenderGains.build());

    RobotVisualization.instance().setExenderSource(logged.extenderAngle);
  }

  /**
   * Sets the speed for the intake
   *
   * @param speed
   */
  public void setIntakeSpeed(AngularVelocity speed) {
    m_IO.setRollerTargetSpeed(speed);
  }

  /**
   * Sets the speed for the intake
   *
   * @param speed
   */
  public void setIntakeAngle(Angle target) {
    m_IO.setExtenderTargetAngle(target);
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
        m_IO.setRollerTargetSpeed(RPM.of(intakeTargetRPM.get()));
        m_IO.setExtenderTargetAngle(Degrees.of(intakeExtenderTargetAngleDown.get()));
        break;
      case OUTTAKING:
        m_IO.setRollerTargetSpeed(RPM.of(-intakeTargetRPM.get()));
        m_IO.setExtenderTargetAngle(Degrees.of(intakeExtenderTargetAngleDown.get()));
        break;
      case IDLE:
        // stop();
        m_IO.setRollerTargetSpeed(RPM.of(0.0));
        m_IO.setExtenderTargetAngle(Degrees.of(intakeExtenderTargetAngleUp.get()));
        break;
    }
    rollerGains.ifGainsHaveChanged((gains) -> this.m_IO.setRollerGains(gains));
    extenderGains.ifGainsHaveChanged((gains) -> this.m_IO.setExtenderGains(gains));
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
          m_IO.setRollerTargetSpeed(RPM.of(rpm.getAsDouble()));
        },
        this);
  }

  public Command getNewSetIntakeExtenderAngleCommand(Supplier<Angle> angle, boolean negate) {
    return new InstantCommand(
        () -> {
          m_IO.setExtenderTargetAngle(
              Degrees.of((negate) ? -1 * angle.get().in(Degrees) : angle.get().in(Degrees)));
        },
        this);
  }
}
