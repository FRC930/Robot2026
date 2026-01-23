package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim implements IndexerIO {

  private Voltage indexerAppliedVoltage = Volts.mutable(0.0);

  private final FlywheelSim indexerSim;

  public IndexerIOSim() {
    indexerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1),
            0.01);
  }

  @Override
  public void setIndexerTarget(Voltage volts) {
    this.indexerAppliedVoltage = volts;
  }

  @Override
  public void stop() {
    this.indexerAppliedVoltage = Volts.zero();
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    inputs.angularVelocity.mut_replace(indexerSim.getAngularVelocity());
    inputs.setVoltage.mut_replace(indexerAppliedVoltage);
    inputs.voltage.mut_replace(Volts.of(indexerSim.getInputVoltage()));

    indexerSim.setInputVoltage(indexerAppliedVoltage.in(Volts));
    indexerSim.update(0.02);
  }
}
