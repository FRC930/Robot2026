package frc.robot.aiming;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
  public static final double TURRET_MIN_DEG = -180.0;
  public static final double TURRET_MAX_DEG = 180.0;

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
      new LoggedTunableNumber("Aiming/flywheelRadiusM", 0.0508);
  public static final LoggedTunableNumber SPEED_TRANSFER_RATIO =
      new LoggedTunableNumber("Aiming/speedTransferRatio", 0.8);
  public static final double SHOOTER_MIN_RPM = 1000.0;
  public static final double SHOOTER_MAX_RPM = 6000.0;

  // ===== HOOD PARAMETERS =====
  public static final double HOOD_MIN_DEG = 15.0;
  public static final double HOOD_MAX_DEG = 65.0;

  // ===== SIMULATION PARAMETERS =====
  public static final double SIM_DT = 0.005;
  public static final double SIM_MAX_TIME = 3.0;
  public static final int TRAJECTORY_MAX_POINTS = 100;

  // ===== LAUNCH ANGLE =====
  // Desired field-frame launch angle (degrees). Higher = more lob, must be within hood range.
  // Ball arrives descending as long as this is above the minimum-energy angle (~45° for level
  // targets).
  public static final LoggedTunableNumber TARGET_LAUNCH_ANGLE_DEG =
      new LoggedTunableNumber("Aiming/targetLaunchAngleDeg", 55.0);

  // ===== VELOCITY COMPENSATION =====
  // Minimum ball radial speed before solution is considered invalid (m/s)
  public static final double MIN_BALL_RADIAL_SPEED = 0.5;
}
