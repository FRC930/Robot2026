package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public enum IndexerState {
  TESTING(RPM.zero(), RPM.zero()),
  IDLE(RPM.zero(), RPM.zero()),
  FEEDING(RPM.of(4.0), RPM.of(4.0));

  private AngularVelocity m_indexerVelocity;
  private AngularVelocity m_feederVelocity;

  private IndexerState(AngularVelocity indexerVelocity, AngularVelocity feederVelocity) {
    m_indexerVelocity = indexerVelocity;
    m_feederVelocity = feederVelocity;
  }

  public AngularVelocity indexerVelocity() {
    return m_indexerVelocity;
  }

  public AngularVelocity feederVelocity() {
    return m_feederVelocity;
  }
}
