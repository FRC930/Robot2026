package frc.robot.subsystems.compactor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotVisualization;
import frc.robot.util.EnumState;
import frc.robot.util.LoggedTunableGainsBuilder;
import org.littletonrobotics.junction.Logger;

public class CompactorSubsystem extends SubsystemBase implements CompactorEvents {
  private CompactorIO m_IO;

  private final EnumState<CompactorState> m_state =
      new EnumState<>("Compactor/States", CompactorState.TOP);

  private CompactorInputsAutoLogged logged = new CompactorInputsAutoLogged();

  public static final double SPOOL_RADIUS = 1.751 / 2.0;

  public static final double INCHES_PER_ROT = (2.0 * Math.PI * SPOOL_RADIUS);

  public static final double REDUCTION = (4.0 / 1.0);

  public LoggedTunableGainsBuilder tunableGains =
      new LoggedTunableGainsBuilder("Gains/Compactor/", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  public CompactorSubsystem(CompactorIO IO) {
    m_IO = IO;

    logged.distance = Inches.mutable(0);
    logged.velocity = InchesPerSecond.mutable(0);
    logged.setPoint = Meters.mutable(0);
    logged.supplyCurrent = Amps.mutable(0);
    logged.torqueCurrent = Amps.mutable(0);
    logged.voltageSetPoint = Volts.mutable(0);
    logged.voltage = Volts.mutable(0);

    RobotVisualization.instance().setCompactorExtensionSource(logged.distance);
  }

  /**
   * @param target
   */
  public void setCompactorHeight(Distance target) {
    m_IO.setCompactorHeight(target);
  }

  public void setTestingState() {
    m_state.set(CompactorState.TESTING);
  }

  public void periodic() {
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/Compactor", logged);
    switch (m_state.get()) {
      case IDLE:
        break;
      case TOP:
        setCompactorHeight(Inches.of(10.0));
        break;
      case BOTTOM:
        setCompactorHeight(Inches.of(0.0));
    }
    tunableGains.ifGainsHaveChanged((gains) -> this.m_IO.setGains(gains));
  }

  public Command idleCommand() {
    return runOnce(() -> m_state.set(CompactorState.IDLE));
  }

  public Command goToTopCommand() {
    return runOnce(() -> m_state.set(CompactorState.TOP));
  }

  public Command goToBottomCommand() {
    return runOnce(() -> m_state.set(CompactorState.BOTTOM));
  }

  @Override
  public Trigger idleTrigger() {
    return m_state.is(CompactorState.IDLE);
  }

  @Override
  public Trigger goToTopTrigger() {
    return m_state.is(CompactorState.TOP);
  }

  @Override
  public Trigger goToBottomTrigger() {
    return m_state.is(CompactorState.BOTTOM);
  }
}
