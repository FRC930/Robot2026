package frc.robot.subsystems.compactor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class CompactorIOSim implements CompactorIO {
  private final ElevatorFeedforward ff = new ElevatorFeedforward(0.0, 0.0, 0.0);

  private final ProfiledPIDController controller =
      new ProfiledPIDController(5.0, 0.0, 0.0, new Constraints(90, 120));

  private final ElevatorSim sim;

  private Distance target = Inches.of(0);
  private MutVoltage appliedVoltage = Volts.mutable(0.0);

  public CompactorIOSim() {
    sim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getKrakenX60Foc(2),
                Pounds.of(45).in(Kilograms),
                Inches.of(CompactorSubsystem.SPOOL_RADIUS).in(Meters),
                CompactorSubsystem.REDUCTION),
            DCMotor.getKrakenX60Foc(2),
            Inches.of(0).in(Meters),
            Inches.of(32).in(Meters),
            true,
            Inches.of(0).in(Meters));
  }

  private void runVolts(Voltage volts) {
    double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
    appliedVoltage.mut_replace(clampedEffort, Volts);
    sim.setInputVoltage(clampedEffort);
  }

  private void updateVoltageSetpoint() {
    Distance currentPosition = Meters.of(sim.getPositionMeters());
    LinearVelocity currentVelocity = MetersPerSecond.of(sim.getVelocityMetersPerSecond());
    Voltage controllerVoltage =
        Volts.of(controller.calculate(currentPosition.in(Inches), this.target.in(Inches)));
    Voltage feedForwardVoltage = Volts.of(ff.calculate(currentVelocity.in(InchesPerSecond)));

    Voltage effort = controllerVoltage.plus(feedForwardVoltage);

    runVolts(effort);
  }

  @Override
  public void updateInputs(CompactorInputs input) {
    sim.update(0.02);
    input.distance.mut_replace(sim.getPositionMeters(), Meters);
    input.velocity.mut_replace(MetersPerSecond.of(sim.getVelocityMetersPerSecond()));
    input.setPoint.mut_replace(Inches.of(controller.getGoal().position));
    input.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    input.torqueCurrent.mut_replace(input.supplyCurrent.in(Amps), Amps);
    input.voltageSetPoint.mut_replace(appliedVoltage);

    updateVoltageSetpoint();
  }

  @Override
  public void setCompactorHeight(Distance target) {
    this.target = target;
    controller.setGoal(target.in(Inches));
  }
}
