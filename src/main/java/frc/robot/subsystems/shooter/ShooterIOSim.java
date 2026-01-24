package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private Voltage shooterAppliedVoltage = Volts.mutable(0.0);

  private final FlywheelSim shooterSim;

  public ShooterIOSim() {
    shooterSim =
        // TODO shooter may not utilize flywheel, keep an eye on
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1),
            0.01);
  }

  @Override
  public void setShooterTarget(Voltage target) {
    this.shooterAppliedVoltage = target;
  }

  @Override
  public void stop() {
    setShooterTarget(Volts.of(0));
  }

  @Override
  public void updateInputs(ShooterInputs input) {
    input.shooterAngularVelocity.mut_replace(shooterSim.getAngularVelocity());
    input.shooterSetVoltage.mut_replace(shooterAppliedVoltage);

    // Periodic
    shooterSim.setInputVoltage(shooterAppliedVoltage.in(Volts));
    shooterSim.update(0.02);
  }
}
