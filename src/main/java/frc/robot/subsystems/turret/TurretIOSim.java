package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.Gains;

public class TurretIOSim implements TurretIO {
  private Angle turretAppliedAngle = Degrees.mutable(0.0);

  private final SingleJointedArmSim turretSim;
  private ArmFeedforward ff = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
  private final ProfiledPIDController controller =
      new ProfiledPIDController(0.3, 0.0, 0.0, new Constraints(360.0, 720.0));

  private static final DCMotor kArmMotor = DCMotor.getKrakenX60(1); // e.g., one NEO motor
  private static final double kGearing = 50.0; // e.g., 50:1 gear ratio
  private static final double kMoI = 1.5; // Moment of inertia in kg/m^2 (from CAD)
  private static final double kArmLength = Units.inchesToMeters(30.0); // e.g., 30 inches long
  private static final double kMinAngle = Units.degreesToRadians(-360); // e.g., -90 degrees
  private static final double kMaxAngle = Units.degreesToRadians(360.0); // e.g., 90 degrees
  private static final boolean kSimulateGravity = false;

  public TurretIOSim() {
    turretSim =
        new SingleJointedArmSim(
            kArmMotor, kGearing, kMoI, kArmLength, kMinAngle, kMaxAngle, kSimulateGravity, 0);
  }

  @Override
  public void stop() {
    this.turretAppliedAngle = Radians.mutable(0);
  }

  @Override
  public void updateInputs(TurretInputs input) {
    Voltage volts = updateTurretPID();
    input.turretAngularVelocity.mut_replace(RadiansPerSecond.of(turretSim.getVelocityRadPerSec()));
    input.turretAngle.mut_replace(turretSim.getAngleRads(), Radians);
    input.turretSetAngle.mut_replace(turretAppliedAngle);
    input.turretVoltage.mut_replace(volts);
    input.turretSupplyCurrent.mut_replace(turretSim.getCurrentDrawAmps(), Amps);
    // inputs.turretTorqueCurrent.mut_replace();

  }

  private Voltage updateTurretPID() {
    Angle currentAngle = Radians.of(turretSim.getAngleRads());

    Voltage controllerVoltage =
        Volts.of(controller.calculate(currentAngle.in(Degrees), turretAppliedAngle.in(Degrees)));
    Voltage feedForwardVoltage =
        Volts.of(
            ff.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity));

    Voltage effort = controllerVoltage.plus(feedForwardVoltage);

    turretSim.setInputVoltage(effort.in(Volts));
    turretSim.update(0.02);
    return effort;
  }

  @Override
  public void setTarget(double position) {
    turretAppliedAngle = Degrees.of(position);
  }

  @Override
  public void setGains(Gains gains) {}

  @Override
  public Angle getCanCoderAngle1() {
    return Degrees.of(0);
  }

  @Override
  public Angle getCanCoderAngle2() {
    return Degrees.of(0);
  }
}
