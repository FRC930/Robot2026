package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotVisualization;
import frc.robot.util.EnumState;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase implements TurretEvents {

  private LoggedTunableNumber IdleAngle = new LoggedTunableNumber("Turret/IdleAngle", 10.00);

  private final TurretIO m_IO;
  private volatile boolean shouldThreadCommand = false;

  private final EnumState<TurretState> m_state =
      new EnumState<>("Turret/States", TurretState.AIMING);

  private static final double VIEW_CHANGE = 0.0;
  private static final double TURRET_MIN_POS = -180.0;
  private static final double TURRET_MAX_POS = 180.0;

  private TurretInputsAutoLogged logged = new TurretInputsAutoLogged();

  private final DoubleSupplier turretAngleSupplier;

  public TurretSubsystem(TurretIO IO, DoubleSupplier turretAngleSupplier) {
    m_IO = IO;
    this.m_IO.setGains(tunableGains.build());
    logged.turretAngle = Degrees.mutable(0);
    logged.canCoderAngle1 = Degrees.mutable(0);
    logged.canCoderAngle2 = Degrees.mutable(0);
    logged.turretSetAngle = Degrees.mutable(0);
    logged.turretAngularVelocity = RPM.mutable(0);
    logged.turretVoltage = Volts.mutable(0);
    logged.turretSupplyCurrent = Amps.mutable(0);
    logged.turretTorqueCurrent = Amps.mutable(0);

    RobotVisualization.instance().setTurretSource(logged.turretAngle);

    this.turretAngleSupplier = turretAngleSupplier;
  }

  public LoggedTunableGainsBuilder tunableGains =
      new LoggedTunableGainsBuilder(
          "Gains/TurretSubsystem/", 170.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  /**
   * Sets the target angle of the subsystem.
   *
   * @param angle The angle in degrees from the horizontal
   */
  public void setPosition(double angle) {
    double clampedPosition = MathUtil.clamp(angle, TURRET_MIN_POS, TURRET_MAX_POS) + VIEW_CHANGE;
    Logger.recordOutput(this.getClass().getSimpleName() + "/ClampedPosition", clampedPosition);
    m_IO.setTarget(clampedPosition);
  }

  public Command idleCommand() {
    return runOnce(
        () -> {
          m_state.set(TurretState.IDLE);
        });
  }

  public Command aimingCommand() {
    return runOnce(
        () -> {
          m_state.set(TurretState.AIMING);
        });
  }

  public Command passingCommand() {
    return runOnce(
        () -> {
          m_state.set(TurretState.PASSING);
        });
  }

  public void setTestingState() {
    m_state.set(TurretState.TESTING);
  }

  public void stop() {
    m_IO.stop();
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/Turret", logged);
    TurretState state = m_state.get();
    shouldThreadCommand = (state == TurretState.AIMING || state == TurretState.PASSING);
    switch (state) {
      case AIMING:
      case PASSING:
        break; // 250Hz thread handles motor commands
      case IDLE:
        setPosition(IdleAngle.get());
        break;
    }
    tunableGains.ifGainsHaveChanged((gains) -> this.m_IO.setGains(gains));
  }

  public boolean shouldThreadCommand() {
    return shouldThreadCommand;
  }

  public TurretIO getIO() {
    return m_IO;
  }

  @Override
  public Trigger isIdleTrigger() {
    return m_state.is(TurretState.IDLE);
  }

  @Override
  public Trigger isPassingTrigger() {
    return m_state.is(TurretState.PASSING);
  }

  public Command getNewSetTurretAngleCommand(DoubleSupplier angle) {
    return new InstantCommand(
        () -> {
          m_IO.setTarget(angle.getAsDouble());
        },
        this);
  }
}
