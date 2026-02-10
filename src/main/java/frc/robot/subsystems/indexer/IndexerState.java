package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public enum IndexerState {
  TESTING(RPM.zero(), Volts.zero()),
  IDLE(RPM.zero(), Volts.zero()),
  FEEDING(RPM.of(4.0), Volts.of(9.0));

  private AngularVelocity m_indexerVelocity;
  private Voltage m_feederVolts;

  private IndexerState(AngularVelocity indexerVelocity, Voltage feederVolts) {
    m_indexerVelocity = indexerVelocity;
    m_feederVolts = feederVolts;
  }

  public AngularVelocity indexerVelocity() {
    return m_indexerVelocity;
  }

  public Voltage feederVolts() {
    return m_feederVolts;
  }
}
