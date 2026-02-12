package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.Gains;

public class TurretIOSim implements TurretIO {
  private Angle turretAppliedAngle = Degrees.mutable(0.0);

  private final SingleJointedArmSim turretSim;

  private static final DCMotor kArmMotor = DCMotor.getKrakenX60(1); // e.g., one NEO motor
  private static final double kGearing = 50.0; // e.g., 50:1 gear ratio
  private static final double kMoI = 1.5; // Moment of inertia in kg/m^2 (from CAD)
  private static final double kArmLength = Units.inchesToMeters(30.0); // e.g., 30 inches long
  private static final double kMinAngle = Units.degreesToRadians(-360.); // e.g., -90 degrees
  private static final double kMaxAngle = Units.degreesToRadians(360.0); // e.g., 90 degrees
  private static final boolean kSimulateGravity = true;

  public TurretIOSim() {
    turretSim =
        new SingleJointedArmSim(
            kArmMotor, kGearing, kMoI, kArmLength, kMinAngle, kMaxAngle, kSimulateGravity, 0);
  }

  @Override
  public void stop() {}

  @Override
  public void updateInputs(TurretInputs input) {
    input.turretAngularVelocity.mut_replace(RadiansPerSecond.of(turretSim.getVelocityRadPerSec()));
    input.turretAngle.mut_replace(turretSim.getAngleRads(), Radians);
    input.turretSetAngle.mut_replace(turretAppliedAngle);
    // input.turretSetAngle.mut_replace(turretAppliedVoltage);
    // TODO fix ^ to angle not volts
    // Periodic
    turretSim.setInput(turretAppliedAngle.in(Radians));
    turretSim.update(0.02);
  }

  @Override
  public void setTarget(double position) {
    // TODO implement set angle on the sim
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
