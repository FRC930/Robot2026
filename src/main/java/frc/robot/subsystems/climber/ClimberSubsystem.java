package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import org.littletonrobotics.junction.Logger;

/** Sets the controless the intake and endexer */
public class ClimberSubsystem extends SubsystemBase implements ClimberEvents {
  public static final double SPOOL_RADIUS = 1.751 / 2.0;

  public static final double INCHES_PER_ROT = (2.0 * Math.PI * SPOOL_RADIUS);

  public static final double REDUCTION = (4.0 / 1.0);
  /** Creates a new ExampleSubsystem. */
  private ClimberIO m_IO;

  private final EnumState<ClimberState> currentGoal =
      new EnumState<>("Climber/States", ClimberState.IDLE);

  private ClimberInputsAutoLogged logged = new ClimberInputsAutoLogged();

  public ClimberSubsystem(ClimberIO IO) {
    m_IO = IO;
    logged.distance = Inches.mutable(0);
    logged.velocity = InchesPerSecond.mutable(0);
    logged.setPoint = Meters.mutable(0);
    logged.supplyCurrent = Amps.mutable(0);
    logged.torqueCurrent = Amps.mutable(0);
    logged.voltageSetPoint = Volts.mutable(0);
    logged.voltage = Volts.mutable(0);
  }
  /**
   * Sets the speed for the climber
   *
   * @param target
   */
  public void setClimberHeight(Distance target) {
    m_IO.setClimberHeight(target);
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/Climber", logged);
    switch (currentGoal.get()) {
      case IDLE:
        setClimberHeight(Inches.of(0));
        break;
      case L1:
        setClimberHeight(Inches.of(2));
        break;
      case L2:
        setClimberHeight(Inches.of(4));
        break;
      case L3:
        setClimberHeight(Inches.of(6));
        break;
    }
  }

  public Command goToIdleCommand() {
    return runOnce(() -> currentGoal.set(ClimberState.IDLE));
  }

  public Command goToL1Command() {
    return runOnce(() -> currentGoal.set(ClimberState.L1));
  }

  public Command goToL2Command() {
    return runOnce(() -> currentGoal.set(ClimberState.L2));
  }

  public Command goToL3Command() {
    return runOnce(() -> currentGoal.set(ClimberState.L3));
  }

  @Override
  public Trigger goToIdleTrigger() {
    return currentGoal.is(ClimberState.IDLE);
  }

  @Override
  public Trigger goToL1Trigger() {
    return currentGoal.is(ClimberState.L1);
  }

  @Override
  public Trigger goToL2Trigger() {
    return currentGoal.is(ClimberState.L2);
  }

  @Override
  public Trigger goToL3Trigger() {
    return currentGoal.is(ClimberState.L3);
  }
}
