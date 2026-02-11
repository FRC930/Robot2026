package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
  private AngularVelocity m_appliedIntakeAngularVelocity = RPM.mutable(0.0);
  private AngularVelocity m_appliedIntakeVelocity = RPM.mutable(0.0);
  private Voltage m_appliedIntakeExtenderVoltage = Volts.mutable(0.0);

  private IntakeInputsAutoLogged logged = new IntakeInputsAutoLogged();

  // physical constants for intake extender (NOT ACCURATE)
  private static final double kArmGearRatio = 100.0;
  private static final double kArmLengthMeters = Units.inchesToMeters(30);
  private static final double kArmMassKg = 5.0;
  private static final double kMinvoltageRads = 0.0;
  private static final double kMaxVoltageRads = 0.0;
  private final double kArmMOI = 1.0 / 3.0 * kArmMassKg * Math.pow(kArmLengthMeters, 2);

  // gains for extender (NOT ACCURATE)
  private static final double kS = 0.0;
  private static final double kV = 0.0;
  private static final double kA = 0.0;

  // TODO fix this later
  // private Angle startingExtenderVoltage = Degrees(0.0);

  private final ProfiledPIDController controller =
      // FILLER VALUES NOT ACCURATE
      new ProfiledPIDController(1.0, 0.0, 0.0, new Constraints(1000, 361));

  private SimpleMotorFeedforward ff;
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
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1),
            0.01);

    ff = new SimpleMotorFeedforward(kS, kV, kA);
    Voltage startingIntakeVoltage = Volts.zero();
    controller.setGoal(startingIntakeVoltage.in(Volts));
    logged.intakeAngularVelocity = RPM.mutable(0);
    // TODO: Create Feed-Forward
    // extenderSim = new SingleJointedArmSim(
    //                   DCMotor.getKrakenX60Foc(1),
    //                   constants.MotorToSensorGearing * constants.SensorToMechanismGearing,
    //                   SingleJointedArmSim.estimateMOI(constants.Length.in(Meters),
    // constants.Weight.in(Kilograms)),
    //                   constants.Length.in(Meters),
    //                   -9999,
    //                   9999,
    //                   false, //TODO: Tune FFs to allow this to be true
    //                   constants.StartingAngle.in(Radians),
    //                   0.001,
    //                   0.001);
    // extenderController = new ProfiledPIDController(constants.SimGains.kP, constants.SimGains.kI,
    // constants.SimGains.kD, new Constraints(constants.MaxVelocity.in(DegreesPerSecond),
    // constants.MaxAcceleration.in(DegreesPerSecondPerSecond)));
    // extenderff = new ArmFeedforward(constants.SimGains.kS, constants.SimGains.kG,
    // constants.SimGains.kV, constants.SimGains.kA);
    // m_Constants = constants;
    // extenderController.setGoal(constants.StartingAngle.in(Degrees));
  }

  /** Updates the applied voltage to drive the arm towards the noted position */
  // TODO: we need to implement the conversion of volts to arm angle

  /** Updates the applied voltage to drive the arm towards the noted position */
  private void updateIntakeSetpoint() {
    AngularVelocity currentVelocity = intakeSim.getAngularVelocity();

    AngularVelocity controllerVelocity = RPM.of(controller.calculate(currentVelocity.in(RPM)));
    AngularVelocity feedForwardVelocity = RPM.of(ff.calculate(controller.getSetpoint().velocity));

    AngularVelocity effort = controllerVelocity.plus(feedForwardVelocity);

    runVelocity(effort);
  }

  /** Updates the applied voltage to drive the arm towards the noted position */
  private void updateIntakeExtenderSetpoint() {
    // AngularVelocity currentVelocity = intakeSim.getAngularVelocity();

    // Voltage controllerVoltage = Volts.of(controller.calculate(currentVelocity.in(RPM)));
    // Voltage feedForwardVoltage =
    //     Volts.of(
    //         ff.calculate(controller.getSetpoint().velocity));

    // Voltage effort = controllerVoltage.plus(feedForwardVoltage);

    Voltage effort = Volts.of(0.0);

    runExtenderVolts(effort);
  }

  /**
   * Sets the applied RPM to the intakeVelocity
   *
   * @param intakeVelocity
   */
  private void runVelocity(AngularVelocity intakeVelocity) {
    this.m_appliedIntakeVelocity = intakeVelocity;
  }

  /**
   * Sets the applied voltage to the volts
   *
   * @param volts
   */
  private void runExtenderVolts(Voltage intakeExtenderVolts) {
    this.m_appliedIntakeExtenderVoltage = intakeExtenderVolts;
  }

  @Override
  public void setIntakeTarget(AngularVelocity target) {
    controller.setGoal(target.in(RPM));
  }

  @Override
  public void setIntakeExtenderTarget(Voltage voltage) {
    controller.setGoal(voltage.in(Volts));
  }

  @Override
  public void stop() {
    // TODO: need to be cleaned up
    // Voltage currentVoltage = Radians.of(intakeExtenderSim.setVoltage(0));
    // controller.reset(currentVoltage.in(Volts));
    setIntakeTarget(RPM.of(0));
    setIntakeExtenderTarget(Volts.of(0));
  }

  @Override
  public void updateInputs(IntakeInputs input) {
    // update inputs

    //  - intake
    input.intakeAngularVelocity.mut_replace(intakeSim.getAngularVelocity());
    input.intakeSetAngularVelocity.mut_replace(RPM.of(controller.getGoal().velocity));
    //  - intake extender
    input.intakeExtenderVoltage.mut_replace(
        Volts.of(
            intakeExtenderSim
                .getInputVoltage())); // TODO: Not sure how to get actual voltage from the motor
    input.intakeExtenderSetVoltage.mut_replace(m_appliedIntakeExtenderVoltage);
    input.intakeExtenderSupplyCurrent.mut_replace(intakeExtenderSim.getCurrentDrawAmps(), Amps);

    updateIntakeSetpoint();
    // Periodic
    intakeSim.setAngularVelocity(m_appliedIntakeVelocity.in(RPM));
    intakeSim.update(0.02);

    updateIntakeExtenderSetpoint();
    intakeExtenderSim.setInputVoltage(m_appliedIntakeExtenderVoltage.in(Volts));
    intakeExtenderSim.update(0.02);
  }
}
