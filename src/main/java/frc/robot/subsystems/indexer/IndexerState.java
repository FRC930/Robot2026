package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public enum IndexerState {
  IDLE(Volts.zero()),
  FEEDING(Volts.of(4.0));

  private Voltage m_motorVolts;

  private IndexerState(Voltage motorVolts) {
    m_motorVolts = motorVolts;
  }

  public Voltage volts() {
    return m_motorVolts;
  }
}
