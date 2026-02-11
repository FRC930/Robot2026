package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSim implements IntakeIO {
  private AngularVelocity m_intakeVelocitySetPoint = RPM.mutable(0.0);
  private Voltage m_ExtenderVoltageSetPoint = Volts.mutable(0.0);

  // physical constants for intake extender (NOT ACCURATE)
  private static final double kArmGearRatio = 100.0;
  private static final double kArmLengthMeters = Units.inchesToMeters(12.0);
  private static final double kArmMassKg = Units.lbsToKilograms(3.0);
  public static final double kMinvoltageRads = Units.degreesToRadians(0.0);
  public static final double kMaxVoltageRads = Units.degreesToRadians(90.0);
  private final double kArmMOI = 1.0 / 3.0 * kArmMassKg * Math.pow(kArmLengthMeters, 2);

  private SingleJointedArmSim extenderSim;
  private ArmFeedforward extenderFF;
  private final ProfiledPIDController extenderPID =
      // FILLER VALUES NOT ACCURATE
      new ProfiledPIDController(0.1, 0.0, 0.0, new Constraints(2.0 * Math.PI, Math.PI));
  // gains for intake (NOT ACCURATE)
  private static final double extenderkS = 0.0;
  private static final double extenderkG = 0.0;
  private static final double extenderkV = 0.0;
  private static final double extenderkA = 0.0;

  // intake Roller
  private final FlywheelSim intakeSim;
  private SimpleMotorFeedforward intakeFF;
  private ProfiledPIDController intakePID =
      new ProfiledPIDController(0.0069, 0.0, 0.0, new Constraints(6000, 10000));
  // gains for intake (NOT ACCURATE)
  private static final double intakekS = 0.0;
  private static final double intakekV = 0.0;
  private static final double intakekA = 0.0;

  public IntakeIOSim() {

    // Intake roller
    intakeSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1),
            0.01);
    intakeFF = new SimpleMotorFeedforward(intakekS, intakekV, intakekA);
    setIntakeTarget(RPM.zero());

    // TODO: Create Feed-Forward
    extenderSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            1.0,
            SingleJointedArmSim.estimateMOI(kArmLengthMeters, kArmMassKg),
            kArmLengthMeters,
            kMinvoltageRads,
            kMaxVoltageRads,
            false, // TODO NOT using gravity may need to switch angles so 0 is down. and 90 is up
            kMinvoltageRads,
            0.001,
            0.001);
    extenderFF = new ArmFeedforward(extenderkS, extenderkG, extenderkV, extenderkA);
    setIntakeExtenderTarget(Volts.of(0.0));
  }

  @Override
  public void setIntakeTarget(AngularVelocity target) {
    m_intakeVelocitySetPoint = target;
  }

  @Override
  public void setIntakeExtenderTarget(Voltage targetVoltage) {
    m_ExtenderVoltageSetPoint = targetVoltage;
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
    updateIntake();
    input.intakeAngularVelocity.mut_replace(intakeSim.getAngularVelocity());
    input.intakeSetAngularVelocity.mut_replace(m_intakeVelocitySetPoint);

    //  - intake extender
    double voltsToExtend = updateExtender();
    input.intakeExtenderVoltage.mut_replace(
        Volts.of(voltsToExtend)); // TODO: Not sure how to get actual voltage from the motor
    input.intakeExtenderSetVoltage.mut_replace(m_ExtenderVoltageSetPoint);
    input.intakeExtenderSupplyCurrent.mut_replace(extenderSim.getCurrentDrawAmps(), Amps);

    input.intakeExtenderAngle.mut_replace(extenderSim.getAngleRads(), Radians);
  }

  private void updateIntake() {
    // Current velocity from simulation
    double currentVelocity = intakeSim.getAngularVelocity().in(RPM);
    double targetVelocity = m_intakeVelocitySetPoint.in(RPM);

    // PID output (in volts) based on velocity error
    double pidOutput = intakePID.calculate(currentVelocity, targetVelocity);

    // Feedforward voltage for target velocity
    double ffOutput = intakeFF.calculate(targetVelocity);
    Logger.recordOutput("TOTALFF", ffOutput);

    // Total voltage command
    double totalVoltage = pidOutput + ffOutput;

    Logger.recordOutput("TOTALVOLTS", totalVoltage);
    // Clamp voltage to [-12V, 12V] to simulate real battery limits
    totalVoltage = Math.max(-12.0, Math.min(12.0, totalVoltage));
    Logger.recordOutput("TOTALVOLTSCLAMP", totalVoltage);
    // Apply voltage to simulation
    intakeSim.setInputVoltage(totalVoltage);

    // Advance simulation
    intakeSim.update(.02);
  }

  private double updateExtender() {
    // Current velocity from simulation
    double currentAngleRads = extenderSim.getAngleRads();
    // NOTE: Assuming if any voltage at maxRad
    double targetAngleRads =
        (m_ExtenderVoltageSetPoint.in(Volts) > 0.0 ? kMaxVoltageRads : kMinvoltageRads);
    Logger.recordOutput(
        "EXTSETANGLE",
        targetAngleRads); // TODO may want in IO to autolog (need to set see for PIDing)

    // PID output (in volts) based on velocity error
    double pidOutput = extenderPID.calculate(currentAngleRads, targetAngleRads);

    // Feedforward voltage for target velocity
    double ffOutput = extenderFF.calculate(targetAngleRads, 0.0); // TODO velocity
    Logger.recordOutput("EXTTOTALFF", ffOutput);

    // Total voltage command
    double totalVoltage = pidOutput + ffOutput;

    Logger.recordOutput("EXTTOTALVOLTS", totalVoltage);
    // Clamp voltage to [-12V, 12V] to simulate real battery limits
    totalVoltage = Math.max(-12.0, Math.min(12.0, totalVoltage));
    Logger.recordOutput("EXTTOTALVOLTSCLAMP", totalVoltage);
    // Apply voltage to simulation
    extenderSim.setInputVoltage(totalVoltage);

    // Advance simulation
    extenderSim.update(.02);
    return totalVoltage;
  }
}
