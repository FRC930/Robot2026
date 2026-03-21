package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public enum IndexerState {
  TESTING(() -> 0.0),
  IDLE(() -> 0.0),
  FEEDING(new LoggedTunableNumber("Indexer/setIndexerPointShooting", 160)),
  REVERSING(new LoggedTunableNumber("Indexer/setIndexerPointReverse", -50)),
  INTAKING(new LoggedTunableNumber("Indexer/setIndexerPointIntaking", 50));

  private DoubleSupplier m_indexerVelocity;

  private IndexerState(DoubleSupplier indexerVelocity) {
    m_indexerVelocity = indexerVelocity;
  }

  public AngularVelocity indexerVelocity() {
    return RPM.of(m_indexerVelocity.getAsDouble());
  }
}
