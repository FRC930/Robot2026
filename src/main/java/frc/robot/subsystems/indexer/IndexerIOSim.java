package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim implements IndexerIO {

  private AngularVelocity indexerAppliedVelocity = RPM.mutable(0.0);
  private Voltage feederAppliedVoltage = Volts.mutable(0.0);

  private final FlywheelSim indexerSim;
  private final FlywheelSim feederSim;

  public IndexerIOSim() {
    indexerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1),
            0.01);
    feederSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1),
            0.01);
  }

  @Override
  public void setIndexerTarget(AngularVelocity velocity) {
    this.indexerAppliedVelocity = velocity;
  }

  @Override
  public void setFeederTarget(Voltage volts) {
    this.feederAppliedVoltage = volts;
  }

  @Override
  public void stop() {
    this.indexerAppliedVelocity = RPM.zero();
    this.feederAppliedVoltage = Volts.zero();
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    // inputs.indexerAngularVelocity.mut_replace(indexerSim.getAngularVelocity());
    inputs.indexerSetpoint.mut_replace(this.indexerAppliedVelocity);
    inputs.indexerAngularVelocity.mut_replace(indexerSim.getAngularVelocity());

    // inputs.feederAngularVelocity.mut_replace(feederSim.getAngularVelocity());
    inputs.feederSetVoltage.mut_replace(feederAppliedVoltage);
    inputs.feederVoltage.mut_replace(Volts.of(feederSim.getInputVoltage()));

    indexerSim.setAngularVelocity(indexerAppliedVelocity.in(RPM));
    indexerSim.update(0.02);

    feederSim.setInputVoltage(feederAppliedVoltage.in(Volts));
    feederSim.update(0.02);
  }
}
