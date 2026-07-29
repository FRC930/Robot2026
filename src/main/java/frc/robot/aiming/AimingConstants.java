package frc.robot.aiming;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.LoggedTunableNumber;

public final class AimingConstants {

  private AimingConstants() {}

  // ===== TURRET GEOMETRY (measured) =====
  // Offset of turret pivot from robot center, robot-frame (x=forward, y=left)
  public static final Translation2d TURRET_OFFSET_FROM_CENTER =
      new Translation2d(Units.inchesToMeters(1.375), 0.0);

  // Height of the turret pivot above the ground
  public static final double TURRET_PIVOT_HEIGHT_METERS = Units.inchesToMeters(24.0);

  // Turret angular limits (degrees) - full [-180, 180] coverage, not continuous rotation
  public static final double TURRET_MIN_DEG = TurretSubsystem.TURRET_MIN_POS;
  public static final double TURRET_MAX_DEG = TurretSubsystem.TURRET_MAX_POS;

  // ===== BALL PHYSICS (tunable for field calibration) =====
  public static final LoggedTunableNumber BALL_MASS_KG =
      new LoggedTunableNumber("Aiming/ballMassKg", 0.27);
  public static final LoggedTunableNumber BALL_RADIUS_M =
      new LoggedTunableNumber("Aiming/ballRadiusM", 0.0508);
  public static final LoggedTunableNumber DRAG_COEFFICIENT =
      new LoggedTunableNumber("Aiming/dragCoefficient", 0.47);
  public static final LoggedTunableNumber LIFT_COEFFICIENT =
      new LoggedTunableNumber("Aiming/liftCoefficient", 0.15);
  public static final double AIR_DENSITY_KG_M3 = 1.225;
  public static final double GRAVITY = 9.81;

  // ===== SHOOTER PARAMETERS (tunable) =====
  public static final LoggedTunableNumber FLYWHEEL_RADIUS_M =
      new LoggedTunableNumber("Aiming/flywheelRadiusM", Units.inchesToMeters(1.5));
  public static final LoggedTunableNumber SPEED_TRANSFER_RATIO =
      new LoggedTunableNumber("Aiming/speedTransferRatio", 0.56);
  public static final double SHOOTER_MIN_RPM = 1000.0;
  public static final double SHOOTER_MAX_RPM = 3500.0;

  // ===== HOOD PARAMETERS =====
  // Hood measures from vertical (0° = straight up), but the aiming algorithm uses
  // 0° = horizontal. Converted: 10° off vertical = 80°, 43° off vertical = 47°.
  // This is correct, it is the ball trajectory & is SUPPOSED to be inverted
  // The hood MAXANGLE should be used to calcute the HOOD_MIN_DEG.
  // The hood MINANGLE should be used to calcute the HOOD_MAX_DEG.
  public static final double HOOD_MIN_DEG = 90.0 - HoodIOTalonFX.MAXANGLE;
  public static final double HOOD_MAX_DEG = 90.0 - HoodIOTalonFX.MINANGLE;

  // ===== SIMULATION PARAMETERS =====
  public static final double SIM_DT = 0.005;
  public static final double SIM_MAX_TIME = 3.0;
  public static final int TRAJECTORY_MAX_POINTS = 100;

  // ===== LAUNCH ANGLE =====
  // Desired field-frame launch angle (degrees). Higher = more lob, must be within hood range.
  // Ball arrives descending as long as this is above the minimum-energy angle (~45° for level
  // targets).
  public static final LoggedTunableNumber TARGET_LAUNCH_ANGLE_DEG =
      new LoggedTunableNumber("Aiming/targetLaunchAngleDeg", 70.0);
  public static final LoggedTunableNumber TARGET_FAR_LAUNCH_ANGLE_DEG =
      new LoggedTunableNumber("Aiming/targetFarLaunchAngleDeg", 66.0);
  public static final LoggedTunableNumber FAR_DISTANCE_THRESHOLD_M =
      new LoggedTunableNumber("Aiming/farDistanceThresholdM", 3.5);
  public static final LoggedTunableNumber FAR_DISTANCE_HYSTERESIS_M =
      new LoggedTunableNumber("Aiming/farDistanceHysteresisM", 0.2);
  public static final LoggedTunableNumber TARGET_PASS_LAUNCH_ANGLE_DEG =
      new LoggedTunableNumber("Aiming/targetPassLaunchAngleDeg", 61.0);

  // ===== PASS TARGET POSITIONS =====
  public static final Translation2d LOW_RED_PASS =
      new Translation2d(Meters.of(15.5), Meters.of(0.9));
  public static final Translation2d HIGH_RED_PASS =
      new Translation2d(Meters.of(15.5), Meters.of(7.1));
  public static final Translation2d LOW_BLUE_PASS =
      new Translation2d(Meters.of(1.1), Meters.of(0.9));
  public static final Translation2d HIGH_BLUE_PASS =
      new Translation2d(Meters.of(1.1), Meters.of(7.1));

  // ===== DISTANCE-BASED RPM SCALING =====
  // Adjusts RPM based on distance: rpm *= 1.0 + rpmDistanceScale * (distance - rpmRefDistance)
  // Positive scale → far shots boosted, close shots reduced
  public static final LoggedTunableNumber RPM_DISTANCE_SCALE =
      new LoggedTunableNumber("Aiming/rpmDistanceScale", 0.0);
  public static final LoggedTunableNumber RPM_REF_DISTANCE_M =
      new LoggedTunableNumber("Aiming/rpmRefDistanceM", 3);

  // ===== VELOCITY COMPENSATION =====
  // Minimum ball radial speed before solution is considered invalid (m/s)
  public static final double MIN_BALL_RADIAL_SPEED = 0.5;

  // ===== HIGH-FREQUENCY LOOP =====
  // Frequency for aiming computation and motor command threads (Hz)
  public static final double AIMING_FREQUENCY = 250.0;
}
