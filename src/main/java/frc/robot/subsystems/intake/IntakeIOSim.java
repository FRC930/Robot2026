package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
  private Voltage intakeAppliedVoltage = Volts.mutable(0.0);
  private Voltage intakeExtenderAppliedVoltage = Volts.mutable(0.0);

  private Voltage m_appliedIntakeVoltage = Volts.mutable(0.0);
  private Voltage m_appliedIntakeExtenderVoltage = Volts.mutable(0.0);

  // physical constants for intake extender (NOT ACCURATE)
  private static final double kArmGearRatio = 100.0;
  private static final double kArmLengthMeters = Units.inchesToMeters(30);
  private static final double kArmMassKg = 5.0;
  private static final double kMinAngleRads = Units.degreesToRadians(0);
  private static final double kMaxAngleRads = Units.degreesToRadians(180);
  private final double kArmMOI = 1.0 / 3.0 * kArmMassKg * Math.pow(kArmLengthMeters, 2);

  // gains for extender (NOT ACCURATE)
  private static final double kS = 0.1;
  private static final double kG = 0.5;
  private static final double kV = 3.0;
  private static final double kA = 0.05;

  private Angle startingExtenderAngle = Degrees.of(0);

  private final ProfiledPIDController controller =
      // FILLER VALUES NOT ACCURATE
      new ProfiledPIDController(0.1, 0.0, 0.0, new Constraints(1000, 361));

  private ArmFeedforward ff;
  private final FlywheelSim intakeSim;
  private final SingleJointedArmSim intakeExtenderSim;

  public IntakeIOSim() {
    intakeSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1),
            0.01);
    intakeExtenderSim =
        // FILLER VALUES NOT ACCURATE
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            kArmGearRatio,
            kArmMOI,
            kArmLengthMeters,
            kMinAngleRads,
            kMaxAngleRads,
            true,
            0);

    ff = new ArmFeedforward(kS, kG, kV, kA);
    controller.setGoal(startingExtenderAngle.in(Degrees));
  }

  /** Updates the applied voltage to drive the arm towards the noted position */
  private void updateVoltageSetpoint() {
    Angle currentExtenderAngle = Radians.of(intakeExtenderSim.getAngleRads());

    Voltage controllerVoltage = Volts.of(controller.calculate(currentExtenderAngle.in(Degrees)));
    Voltage feedForwardVoltage =
        Volts.of(
            ff.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity));

    Voltage effort = controllerVoltage.plus(feedForwardVoltage);

    runVolts(effort);
  }

  /**
   * Sets the applied voltage to the volts
   *
   * @param volts
   */
  private void runVolts(Voltage intakeExtenderVolts) {
    this.m_appliedIntakeExtenderVoltage = intakeExtenderVolts;
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
    Angle currentAngle = Radians.of(intakeExtenderSim.getAngleRads());
    controller.reset(currentAngle.in(Degrees));
    setIntakerTarget(Volts.of(0));
    setIntakerExtenderTarget(Degrees.of(0));
  }

  @Override
  public void updateInputs(IntakeInputs input) {
    // update inputs

    //  - intake
    input.intakeAngularVelocity.mut_replace(
        DegreesPerSecond.convertFrom(intakeExtenderSim.getVelocityRadPerSec(), RadiansPerSecond),
        DegreesPerSecond);
    input.intakeVoltage.mut_replace(m_appliedIntakeVoltage);
    input.intakeSetVoltage.mut_replace(m_appliedIntakeVoltage);
    //  - intake extender
    input.intakeExtenderVoltage.mut_replace(m_appliedIntakeExtenderVoltage);
    input.intakeExtenderSetVoltage.mut_replace(m_appliedIntakeExtenderVoltage);
    input.intakeExtenderSupplyCurrent.mut_replace(intakeExtenderSim.getCurrentDrawAmps(), Amps);
    input.intakeExtenderTorqueCurrent.mut_replace(input.intakeExtenderSupplyCurrent.in(Amps), Amps);
    input.intakeExtenderSetPoint.mut_replace(controller.getGoal().position, Degrees);
    input.intakeExtenderAngle.mut_replace(
        Degrees.convertFrom(intakeExtenderSim.getAngleRads(), Radians), Degrees);

    // Periodic
    intakeSim.setInputVoltage(intakeAppliedVoltage.in(Volts));
    intakeSim.update(0.02);

    intakeExtenderSim.setInputVoltage(intakeExtenderAppliedVoltage.in(Volts));
    intakeSim.update(0.02);
  }
}
