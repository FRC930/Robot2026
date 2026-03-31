// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotVisualization;
import frc.robot.aiming.AimingService;
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

  public static final double INTAKE_EXTENDER_ANGLE_UP = 83.0;

  private double rpm =
      (11.0 / 12.0)
          * IntakeIOTalonFX.KRACKEN_X60_FOC_MAX_RPM.in(RPM)
          / IntakeIOTalonFX.GEAR_RATIO_ROLLERS;

  private LoggedTunableNumber intakeTargetRPM =
      new LoggedTunableNumber("Intake/intakeTargetRPM", 3000);
  private LoggedTunableNumber intakeExtenderTargetAngleUp =
      new LoggedTunableNumber("Intake/intakeExtenderTargetAngleUp", INTAKE_EXTENDER_ANGLE_UP);
  private LoggedTunableNumber intakeExtenderTargetAngleDown =
      new LoggedTunableNumber("Intake/intakeExtenderTargetAngleDown", 0.0);
  private final EnumState<IntakeState> currentGoal =
      new EnumState<>("Intake/States", IntakeState.IDLE);

  // Agitation tunables
  private static final LoggedTunableNumber agitateDownAngle =
      new LoggedTunableNumber("Intake/agitateDownAngle", 20.0);
  private static final LoggedTunableNumber agitateUpAngle =
      new LoggedTunableNumber("Intake/agitateUpAngle", 60.0);
  private static final LoggedTunableNumber agitateMaxVelocity =
      new LoggedTunableNumber("Intake/agitateMaxVelocityDegPerSec", 800.0);
  private static final LoggedTunableNumber agitateMaxAcceleration =
      new LoggedTunableNumber("Intake/agitateMaxAccelDegPerSec2", 1000.0);
  private static final double AGITATE_POSITION_TOLERANCE_DEG = 2.0;

  // Agitation state
  private TrapezoidProfile agitateProfile;
  private TrapezoidProfile.State agitateCurrentState = new TrapezoidProfile.State(0, 0);
  private double agitateGoalPosition;
  private boolean agitateInitialized = false;

  public LoggedTunableGainsBuilder rollerGains =
      new LoggedTunableGainsBuilder(
          "Gains/IntakeSubsystem/", 1.0, 0, 0.1, 0.33, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  public LoggedTunableGainsBuilder extenderGains =
      new LoggedTunableGainsBuilder(
          "Gains/ExtenderSubsystem/", 200.0, 0, 0.0, 0.0, 0.425, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

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
    m_IO.setRollerGains(rollerGains.build());
    logged.numberFuelHave = 0;
    m_IO.setExtenderGains(extenderGains.build());
    RobotVisualization.instance().setExtenderTwistSource(logged.extenderAngle);
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

  public Command shootingCommand() {
    return runOnce(
        () -> {
          currentGoal.set(IntakeState.SHOOTING);
        });
  }

  public Command raisedCommand() {
    return runOnce(
        () -> {
          currentGoal.set(IntakeState.RAISED);
        });
  }

  public Command agitateCommand() {
    return runOnce(
        () -> {
          currentGoal.set(IntakeState.AGITATING);
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
      case SHOOTING:
        if (Constants.currentMode == Constants.Mode.SIM) {
          if (m_IO instanceof IntakeIOSim) {
            IntakeIOSim sim = (IntakeIOSim) m_IO;
            sim.shootFuel();
            sim.setRunning(false);
          }
        }
        break;
      case INTAKING:
        agitateInitialized = false;
        m_IO.setRollerTargetSpeed(RPM.of(intakeTargetRPM.get()));
        m_IO.setExtenderTargetAngle(Degrees.of(intakeExtenderTargetAngleDown.get()));
        if (Constants.currentMode == Constants.Mode.SIM) {
          if (m_IO instanceof IntakeIOSim) {
            IntakeIOSim sim = (IntakeIOSim) m_IO;
            sim.setRunning(true);
          }
        }
        break;
      case OUTTAKING:
        agitateInitialized = false;
        m_IO.setRollerTargetSpeed(RPM.of(-intakeTargetRPM.get()));
        m_IO.setExtenderTargetAngle(Degrees.of(intakeExtenderTargetAngleDown.get()));
        if (Constants.currentMode == Constants.Mode.SIM) {
          AimingService.trajectorySim.setSpawnFuelOnGround(false);
          if (m_IO instanceof IntakeIOSim) {
            IntakeIOSim sim = (IntakeIOSim) m_IO;
            sim.setRunning(false);
          }
        }
        break;
      case RAISED:
        m_IO.setRollerTargetSpeed(RPM.of(intakeTargetRPM.get()));
        m_IO.setExtenderTargetAngle(Degrees.of(intakeExtenderTargetAngleUp.get()));
        if (Constants.currentMode == Constants.Mode.SIM) {
          AimingService.trajectorySim.setSpawnFuelOnGround(false);
          if (m_IO instanceof IntakeIOSim) {
            IntakeIOSim sim = (IntakeIOSim) m_IO;
            sim.setRunning(false);
          }
        }
        break;
      case IDLE:
        agitateInitialized = false;
        stop();
        if (Constants.currentMode == Constants.Mode.SIM) {
          AimingService.trajectorySim.setSpawnFuelOnGround(false);
          if (m_IO instanceof IntakeIOSim) {
            IntakeIOSim sim = (IntakeIOSim) m_IO;
            sim.setRunning(false);
          }
        }
        // m_IO.setRollerTargetSpeed(RPM.of(0.0));
        // m_IO.setExtenderTargetAngle(Degrees.of(intakeExtenderTargetAngleUp.get()));
        // TODO add a up state for when not intaking and not outtaking
        break;
      case AGITATING:
        m_IO.stop();
        updateAgitation();
        if (Constants.currentMode == Constants.Mode.SIM) {
          if (m_IO instanceof IntakeIOSim) {
            IntakeIOSim sim = (IntakeIOSim) m_IO;
            sim.shootFuel();
            sim.setRunning(false);
          }
        }
        break;
    }
    rollerGains.ifGainsHaveChanged((gains) -> this.m_IO.setRollerGains(gains));
    extenderGains.ifGainsHaveChanged((gains) -> this.m_IO.setExtenderGains(gains));
  }

  private void updateAgitation() {
    if (!agitateInitialized) {
      agitateProfile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  agitateMaxVelocity.get(), agitateMaxAcceleration.get()));
      agitateCurrentState = new TrapezoidProfile.State(logged.extenderAngle.in(Degrees), 0);
      agitateGoalPosition = agitateDownAngle.get();
      agitateInitialized = true;
    }

    TrapezoidProfile.State goal = new TrapezoidProfile.State(agitateGoalPosition, 0);
    agitateCurrentState = agitateProfile.calculate(0.020, agitateCurrentState, goal);

    m_IO.setExtenderTargetAngle(Degrees.of(agitateCurrentState.position));

    if (Math.abs(agitateCurrentState.position - agitateGoalPosition)
        < AGITATE_POSITION_TOLERANCE_DEG) {
      if (agitateGoalPosition == agitateDownAngle.get()) {
        agitateGoalPosition = agitateUpAngle.get();
      } else {
        agitateGoalPosition = agitateDownAngle.get();
      }
    }

    Logger.recordOutput("Intake/AgitateSetpoint", agitateCurrentState.position);
    Logger.recordOutput("Intake/AgitateGoal", agitateGoalPosition);
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

  @Override
  public Trigger isShootingTrigger() {
    return currentGoal.is(IntakeState.SHOOTING);
  }

  @Override
  public Trigger isRaisedTrigger() {
    return currentGoal.is(IntakeState.RAISED);
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
