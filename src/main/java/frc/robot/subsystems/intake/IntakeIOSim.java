package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {

  // physical constants for intake extender (NOT ACCURATE)
  private static final double kArmGearRatio = 1.0;
  private static final double kArmLengthMeters = Units.inchesToMeters(12.0);
  private static final double kArmMassKg = Units.lbsToKilograms(3.0);
  public static final double kMinExtenderRads = Units.degreesToRadians(0.0);
  public static final double kMaxExtenderRads =
      Units.degreesToRadians(IntakeSubsystem.INTAKE_EXTENDER_ANGLE_UP);

  // intake extender stow (up) angle setpoint, in radians (0 is down, positive is up)
  private Angle m_extenderAngleSetPoint = Radians.mutable(kMaxExtenderRads);

  private SingleJointedArmSim extenderArmSim;
  private ArmFeedforward extenderFF = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
  private final ProfiledPIDController extenderPID =
      new ProfiledPIDController(0.1, 0.0, 0.0, new Constraints(2.0 * Math.PI, Math.PI));

  // intake Roller
  private AngularVelocity m_rollerVelocitySetPoint = RPM.mutable(0.0);

  private final FlywheelSim rollerFlyWheelSim;
  private SimpleMotorFeedforward rollerFF = new SimpleMotorFeedforward(0.0, 0.002, 0.0);
  // NOTE: ProfilePID sorta worked if did not have any FF KV BUT did not reach goal
  // private ProfiledPIDController rollerPID =
  //     new ProfiledPIDController(0.0069, 0.0, 0.0, new Constraints(6000, 10000));
  private PIDController rollerPID = new PIDController(0.0031, 0.0, 0.0);

  public IntakeIOSim() {

    // Setup Intake roller
    rollerFlyWheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0005, 1),
            DCMotor.getKrakenX60Foc(1),
            0.01);

    // Setup Intake extender is an ARM in simulator but it REAL using Voltage and current limits
    extenderArmSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            kArmGearRatio,
            SingleJointedArmSim.estimateMOI(kArmLengthMeters, kArmMassKg),
            kArmLengthMeters,
            kMinExtenderRads,
            kMaxExtenderRads,
            false, // TODO NOT using gravity may need to switch angles so 0 is down. and 90 is up
            kMaxExtenderRads, // make sure to set m_extenderAngleSetPoint to this angle
            0.001,
            0.001);
  }

  @Override
  public void setRollerTargetSpeed(AngularVelocity rpm) {
    m_rollerVelocitySetPoint = rpm;
  }

  @Override
  public void setExtenderTargetAngle(Angle targetAngle) {
    m_extenderAngleSetPoint = targetAngle;
  }

  @Override
  public void stop() {
    // If REAL robot use coast
    setRollerTargetSpeed(RPM.of(0));
    // Doing nothing with extender motor
    // setExtenderTargetAngle(Degrees.of(0));
  }

  @Override
  public void updateInputs(IntakeInputs input) {
    // update inputs

    //  - intake
    updateRollerPID();
    input.rollerVelocity.mut_replace(rollerFlyWheelSim.getAngularVelocity());
    input.rollerVelocitySetPoint.mut_replace(m_rollerVelocitySetPoint);
    input.rollerSupplyCurrent.mut_replace(rollerFlyWheelSim.getCurrentDrawAmps(), Amps);

    //  - intake extender
    // NOTE: Special case given inputing volts to controller down(+)/up(-) intake extender can not
    // find a way to get volts from simulated motor
    double voltsToExtend = updateExtenderPID();
    input.extenderVoltage.mut_replace(
        Volts.of(voltsToExtend)); // TODO: Not sure how to get actual voltage from the motor
    input.extenderAngleSetPoint.mut_replace(m_extenderAngleSetPoint);
    input.extenderSupplyCurrent.mut_replace(extenderArmSim.getCurrentDrawAmps(), Amps);
    input.extenderAngle.mut_replace(extenderArmSim.getAngleRads(), Radians);
  }

  private void updateRollerPID() {
    // Current velocity from simulation
    double currentVelocity = rollerFlyWheelSim.getAngularVelocity().in(RPM);
    double targetVelocity = m_rollerVelocitySetPoint.in(RPM);

    // PID output (in volts) based on velocity error
    double pidOutput = rollerPID.calculate(currentVelocity, targetVelocity);

    // Feedforward voltage for target velocity
    double ffOutput = rollerFF.calculate(targetVelocity);
    // Logger.recordOutput("TOTALFF", ffOutput);

    // Total voltage command
    double totalVoltage = pidOutput + ffOutput;

    // Logger.recordOutput("TOTALVOLTS", totalVoltage);
    // Clamp voltage to [-12V, 12V] to simulate real battery limits
    totalVoltage = Math.max(-12.0, Math.min(12.0, totalVoltage));
    // Logger.recordOutput("TOTALVOLTSCLAMP", totalVoltage);
    // Apply voltage to simulation
    rollerFlyWheelSim.setInputVoltage(totalVoltage);

    // Advance simulation
    rollerFlyWheelSim.update(.02);
  }

  private double updateExtenderPID() {
    // Current velocity from simulation
    double currentAngleRads = extenderArmSim.getAngleRads();
    // NOTE: Assuming if any voltage at maxRad
    double targetAngleRads = m_extenderAngleSetPoint.in(Radians);

    // Logger.recordOutput(
    //     "EXTSETANGLE",
    //     targetAngleRads);

    // PID output (in volts) based on velocity error
    double pidOutput = extenderPID.calculate(currentAngleRads, targetAngleRads);

    // Feedforward voltage for target velocity
    double ffOutput = extenderFF.calculate(targetAngleRads, 0.0); // TODO velocity
    // Logger.recordOutput("EXTTOTALFF", ffOutput);

    // Total voltage command
    double totalVoltage = pidOutput + ffOutput;

    // Logger.recordOutput("EXTTOTALVOLTS", totalVoltage);
    // Clamp voltage to [-12V, 12V] to simulate real battery limits
    totalVoltage = Math.max(-12.0, Math.min(12.0, totalVoltage));
    // Logger.recordOutput("EXTTOTALVOLTSCLAMP", totalVoltage);
    // Apply voltage to simulation
    extenderArmSim.setInputVoltage(totalVoltage);

    // Advance simulation
    extenderArmSim.update(.02);
    return totalVoltage;
  }
}
