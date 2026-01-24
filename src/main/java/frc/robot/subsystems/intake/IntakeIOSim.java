package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
  private Voltage intakeAppliedVoltage = Volts.mutable(0.0);

  private final ProfiledPIDController controller =
      // FILLER VALUES NOT ACCURATE
      new ProfiledPIDController(0.1, 0.0, 0.0, new Constraints(1000, 361));

  private final FlywheelSim intakeSim;
  private final FlywheelSim intakeExtenderSim;

  public IntakeIOSim() {
    intakeSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1),
            0.01);
    intakeExtenderSim =
        // FILLER VALUES NOT ACCURATE
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.28616, 1),
            DCMotor.getKrakenX60Foc(1), 0.001);
  }

  @Override
  public void setIntakerTarget(Voltage target) {
    this.intakeAppliedVoltage = target;
  }

  @Override
  public void setIntakerExtenderTarget(Angle angle) {
    controller.setGoal(angle.in(Degrees));
  }

  @Override
  public void stop() {
    Angle currentAngle = Radians.of(intakeExtenderSim.getOutput(0));
    setIntakerTarget(Volts.of(0));
    controller.reset(currentAngle.in(Degrees));
  }

  @Override
  public void updateInputs(IntakeInputs input) {
    input.intakeAngularVelocity.mut_replace(intakeSim.getAngularVelocity());
    input.intakeSetVoltage.mut_replace(intakeAppliedVoltage);

    // Periodic
    intakeSim.setInputVoltage(intakeAppliedVoltage.in(Volts));
    intakeSim.update(0.02);
  }
}
