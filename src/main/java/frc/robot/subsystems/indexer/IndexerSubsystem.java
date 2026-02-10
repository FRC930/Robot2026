package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase implements IndexerEvents {

  private final IndexerIO m_IO;

  private final EnumState<IndexerState> m_state =
      new EnumState<>("Indexer/State", IndexerState.IDLE);

  private final IndexerInputsAutoLogged m_logged = new IndexerInputsAutoLogged();

  public IndexerSubsystem(IndexerIO IO) {
    m_IO = IO;

    m_logged.indexerSupplyCurrent = Amps.mutable(0);
    m_logged.indexerAngularVelocity = RPM.mutable(0);
    m_logged.indexerSetpoint = RPM.mutable(0);

    m_logged.feederSupplyCurrent = Amps.mutable(0);
    m_logged.feederVoltage = Volts.mutable(0);
    m_logged.feederSetVoltage = Volts.mutable(0);
  }

  public void setTestingState() {
    m_state.set(IndexerState.TESTING);
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
        m_IO.setFeederTarget(this.m_state.get().feederVolts());
        break;
    }
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
          m_IO.setIndexerTarget(AngularVelocity.ofBaseUnits(velocity.getAsDouble(), RPM));
        },
        this);
  }
}
