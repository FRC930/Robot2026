package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import frc.robot.util.LoggedTunableGainsBuilder;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase implements IndexerEvents {

  private final IndexerIO m_IO;

  private final EnumState<IndexerState> m_state =
      new EnumState<>("Indexer/State", IndexerState.IDLE);

  private final IndexerInputsAutoLogged m_logged = new IndexerInputsAutoLogged();

  private final LoggedTunableGainsBuilder m_feederTunableGains =
      new LoggedTunableGainsBuilder(
          "Gains/Feeder/", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  private final LoggedTunableGainsBuilder m_indexerTunableGains =
      new LoggedTunableGainsBuilder(
          "Gains/Indexer/", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  public IndexerSubsystem(IndexerIO IO) {
    m_IO = IO;

    m_logged.indexerSupplyCurrent = Amps.mutable(0);
    m_logged.indexerVelocity = RPM.mutable(0);
    m_logged.indexerSetPoint = RPM.mutable(0);
    m_logged.indexerVoltage = Volts.mutable(0);
    m_IO.setIndexerGains(m_indexerTunableGains.build());

    m_logged.feederSupplyCurrent = Amps.mutable(0);
    m_logged.feederVelocity = RPM.mutable(0);
    m_logged.feederSetPoint = RPM.mutable(0);
    m_logged.feederVoltage = Volts.mutable(0);
    m_IO.setFeederGains(m_feederTunableGains.build());
  }

  public void setTestingState() {
    m_state.set(IndexerState.TESTING);
  }

  public void stop() {
    m_IO.stop();
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(m_logged);
    Logger.processInputs("RobotState/Indexer", m_logged);

    switch (this.m_state.get()) {
      case IDLE:
        m_IO.stop();
        break;
      case FEEDING:
        m_IO.setIndexerTarget(this.m_state.get().indexerVelocity());
        m_IO.setFeederTarget(this.m_state.get().feederVelocity());
        break;
    }
    m_feederTunableGains.ifGainsHaveChanged((gains) -> this.m_IO.setFeederGains(gains));
    m_indexerTunableGains.ifGainsHaveChanged((gains) -> this.m_IO.setIndexerGains(gains));
  }

  @Override
  public Trigger isIdleTrigger() {
    return m_state.is(IndexerState.IDLE);
  }

  @Override
  public Trigger isIndexingTrigger() {
    return m_state.is(IndexerState.FEEDING);
  }

  public Command idleCommand() {
    return runOnce(() -> m_state.set(IndexerState.IDLE));
  }

  public Command indexingCommand() {
    return runOnce(() -> m_state.set(IndexerState.FEEDING));
  }

  public Command getNewSetIndexerVelocityCommand(DoubleSupplier velocity) {
    return new InstantCommand(
        () -> {
          m_IO.setIndexerTarget(RPM.of(velocity.getAsDouble()));
        },
        this);
  }

  public Command getNewSetFeederVelocityCommand(DoubleSupplier velocity) {
    return new InstantCommand(
        () -> {
          m_IO.setFeederTarget(RPM.of(velocity.getAsDouble()));
        },
        this);
  }
}
