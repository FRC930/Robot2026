package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.LoggedTunableNumber;

public enum IndexerState {
  TESTING(() -> 0.0, () -> 0.0),
  IDLE(() -> 0.0 , () ->  0.0),
  FEEDING( // TODO get log tunable numbers to work
      new LoggedTunableNumber("Indexer/setIndexerPoint", 100),
      new LoggedTunableNumber("Indexer/setFeederPoint", 1000));

  private DoubleSupplier m_indexerVelocity;
  private DoubleSupplier m_feederVelocity;

  private IndexerState(DoubleSupplier indexerVelocity, DoubleSupplier feederVelocity) {
    m_indexerVelocity = indexerVelocity;
    m_feederVelocity = feederVelocity;
  }

  public AngularVelocity indexerVelocity() {
    return  RPM.of(m_indexerVelocity.getAsDouble());
  }

  public AngularVelocity feederVelocity() {
    return RPM.of(m_feederVelocity.getAsDouble());
  }
}
