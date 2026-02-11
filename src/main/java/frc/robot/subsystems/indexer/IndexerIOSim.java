package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim implements IndexerIO {

  private AngularVelocity indexerAppliedVelocity = RPM.mutable(0.0);
  private AngularVelocity feederAppliedVelocity = RPM.mutable(0.0);

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
  public void setFeederTarget(AngularVelocity velocity) {
    this.feederAppliedVelocity = velocity;
  }

  @Override
  public void stop() {
    this.indexerAppliedVelocity = RPM.zero();
    this.feederAppliedVelocity = RPM.zero();
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    inputs.indexerSetPoint.mut_replace(this.indexerAppliedVelocity);
    inputs.indexerVelocity.mut_replace(indexerSim.getAngularVelocity());
    inputs.indexerVoltage.mut_replace(Volts.of(indexerSim.getInputVoltage()));

    inputs.feederVelocity.mut_replace(feederSim.getAngularVelocity());
    inputs.feederSetPoint.mut_replace(feederAppliedVelocity);
    inputs.feederVoltage.mut_replace(Volts.of(feederSim.getInputVoltage()));

    // TODO INDEXER simulate using PIDS and volts (see intakeIOSim)
    indexerSim.setAngularVelocity(indexerAppliedVelocity.in(RadiansPerSecond));
    indexerSim.update(0.02);

    // TODO INDEXER simulate using PIDS and volts (see intakeIOSim)
    feederSim.setAngularVelocity(feederAppliedVelocity.in(RadiansPerSecond));
    feederSim.update(0.02);
  }
}
