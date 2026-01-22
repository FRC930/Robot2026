// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import org.littletonrobotics.junction.Logger;

/** Sets the controless the intake and endexer */
public class IntakeSubsystem extends SubsystemBase implements IntakeEvents {
  /** Creates a new ExampleSubsystem. */
  private IntakeIO m_IO;

  private final EnumState<IntakeState> currentGoal =
      new EnumState<>("Intake/States", IntakeState.IDLE);

  private IntakeInputsAutoLogged logged = new IntakeInputsAutoLogged();

  public IntakeSubsystem(IntakeIO IO) {
    m_IO = IO;
    logged.intakeAngularVelocity = DegreesPerSecond.mutable(0);
    logged.intakeVoltage = Volts.mutable(0);
    logged.intakeSetVoltage = Volts.mutable(0);
  }
  /**
   * Sets the speed for the intake
   *
   * @param speed
   */
  public void setIntakeSpeed(Voltage speed) {
    m_IO.setIntakerTarget(speed);
  }
  
  public Command intakeCommand() {
    return runOnce(
        () -> {
          setIntakeSpeed(Volts.of(11.0));
        });
  }

  public Command idleCommand() {
    return runOnce(
        () -> {
          stop();
        });
  }

  public void stop() {
    m_IO.stop();
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/Intake", logged);
  }

  @Override
  public Trigger isIdleTrigger() {
    return currentGoal.is(IntakeState.IDLE);
  }

  @Override
  public Trigger isIntakingTrigger() {
    return currentGoal.is(IntakeState.LAUNCHING);
  }
}
