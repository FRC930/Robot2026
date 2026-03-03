package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public enum IndexerState {
  TESTING(() -> 0.0, () -> 0.0),
  IDLE(() -> 0.0, () -> 0.0),
  FEEDING(
      new LoggedTunableNumber("Indexer/setIndexerPoint", 100),
      new LoggedTunableNumber("Indexer/setFeederPoint", 4000)),
  REVERSING(
      new LoggedTunableNumber("Indexer/setIndexerPointReverse", -100),
      new LoggedTunableNumber("Indexer/setFeederPointReverse", -1000));

  private DoubleSupplier m_indexerVelocity;
  private DoubleSupplier m_feederVelocity;

  private IndexerState(DoubleSupplier indexerVelocity, DoubleSupplier feederVelocity) {
    m_indexerVelocity = indexerVelocity;
    m_feederVelocity = feederVelocity;
  }

  public AngularVelocity indexerVelocity() {
    return RPM.of(m_indexerVelocity.getAsDouble());
  }

  public AngularVelocity feederVelocity() {
    return RPM.of(m_feederVelocity.getAsDouble());
  }
}
