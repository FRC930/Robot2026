package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase implements IndexerEvents {

  private final IndexerIO m_IO;

  private final EnumState<IndexerState> m_state =
      new EnumState<>("Indexer/State", IndexerState.IDLE);

  private final IndexerInputsAutoLogged m_logged = new IndexerInputsAutoLogged();

  public IndexerSubsystem(IndexerIO IO) {
    m_IO = IO;

    m_logged.angularVelocity = DegreesPerSecond.mutable(0);
    m_logged.voltage = Volts.mutable(0);
    m_logged.setVoltage = Volts.mutable(0);
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(m_logged);
    Logger.processInputs("RobotState/Indexer", m_logged);

    switch (this.m_state.get()) {
      case IDLE:
        m_IO.stop();
      default:
        m_IO.setIndexerTarget(this.m_state.get().volts());
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
}
