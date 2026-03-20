package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim implements IndexerIO {

  private AngularVelocity indexerAppliedVelocity = RPM.mutable(0.0);
  private final FlywheelSim indexerSim;
  private SimpleMotorFeedforward indexerFF = new SimpleMotorFeedforward(0.0, 0.002, 0.0);
  private PIDController indexerPID = new PIDController(0.0031, 0.0, 0.0);

  public IndexerIOSim() {
    indexerSim =
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
  public void stop() {
    this.indexerAppliedVelocity = RPM.zero();
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    inputs.indexerSetPoint.mut_replace(this.indexerAppliedVelocity);
    inputs.indexerVelocity.mut_replace(indexerSim.getAngularVelocity());
    inputs.indexerVoltage.mut_replace(Volts.of(indexerSim.getInputVoltage()));

    updateIndexerPID();
  }

  private void updateIndexerPID() {
    // Current velocity from simulation
    double currentVelocity = indexerSim.getAngularVelocity().in(RPM);
    double targetVelocity = indexerAppliedVelocity.in(RPM);

    // PID output (in volts) based on velocity error
    double pidOutput = indexerPID.calculate(currentVelocity, targetVelocity);

    // Feedforward voltage for target velocity
    double ffOutput = indexerFF.calculate(targetVelocity);

    // Total voltage command
    double totalVoltage = pidOutput + ffOutput;

    // Clamp voltage to [-12V, 12V] to simulate real battery limits
    totalVoltage = Math.max(-12.0, Math.min(12.0, totalVoltage));
    // Apply voltage to simulation
    indexerSim.setInputVoltage(totalVoltage);

    // Advance simulation
    indexerSim.update(.02);
  }
}
