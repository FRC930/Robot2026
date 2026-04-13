package frc.robot.aiming;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.hood.HoodIOTalonFX;

public final class AimingConstants {

  private AimingConstants() {}

  // ===== SHOOTER GEOMETRY (measured) =====
  // Offset of shooter pivot from robot center, robot-frame (x=forward, y=left)
  public static final Translation2d SHOOTER_OFFSET_FROM_CENTER =
      new Translation2d(Units.inchesToMeters(-5.93), 0.0);

  // Robot rotation angular limits (degrees) - full [-180, 180] since robot can face any direction
  public static final double ROBOT_AIM_MIN_DEG = -180.0;
  public static final double ROBOT_AIM_MAX_DEG = 180.0;

  public static final double GRAVITY = 9.81;

  // ===== SHOOTER PARAMETERS (tunable) =====
  public static final double FLYWHEEL_RADIUS_M = Units.inchesToMeters(2.0);
  public static final double SPEED_TRANSFER_RATIO = 0.68;
  public static final double SHOOTER_MIN_RPM = 1000.0;
  public static final double SHOOTER_MAX_RPM = 6000.0;

  // ===== HOOD PARAMETERS =====
  // Hood measures from vertical (0° = straight up), but the aiming algorithm uses
  // 0° = horizontal. Converted: 10° off vertical = 80°, 43° off vertical = 47°.
  // This is correct, it is the ball trajectory & is SUPPOSED to be inverted
  // The hood MAXANGLE should be used to calcute the HOOD_MIN_DEG.
  // The hood MINANGLE should be used to calcute the HOOD_MAX_DEG.
  public static final double HOOD_MIN_DEG = 90.0 - HoodIOTalonFX.MAXANGLE;
  public static final double HOOD_MAX_DEG = 90.0 - HoodIOTalonFX.MINANGLE;

  // ===== INTERPOLATION TABLES =====
  // Distance (meters) → hood angle (degrees from horizontal) and shooter RPM.
  // Seed values generated from physics model with updated transfer ratio (0.68).
  // Tune empirically on the real robot.
  public static final InterpolatingDoubleTreeMap HUB_HOOD_ANGLE_MAP =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap HUB_RPM_MAP = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap PASS_HOOD_ANGLE_MAP =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap PASS_RPM_MAP = new InterpolatingDoubleTreeMap();

  static {
    // Hub target: distance (m) → hood angle (deg from horizontal)
    // Seeded at 64° (known good at 7ft). Tune per-distance on robot.
    HUB_HOOD_ANGLE_MAP.put(Units.feetToMeters(3), 64.0);
    HUB_HOOD_ANGLE_MAP.put(Units.feetToMeters(4), 64.0);
    HUB_HOOD_ANGLE_MAP.put(Units.feetToMeters(5), 64.0);
    HUB_HOOD_ANGLE_MAP.put(Units.feetToMeters(6), 64.0);
    HUB_HOOD_ANGLE_MAP.put(Units.feetToMeters(7), 64.0);
    HUB_HOOD_ANGLE_MAP.put(Units.feetToMeters(8), 64.0);
    HUB_HOOD_ANGLE_MAP.put(Units.feetToMeters(9), 64.0);
    HUB_HOOD_ANGLE_MAP.put(Units.feetToMeters(10), 64.0);
    HUB_HOOD_ANGLE_MAP.put(Units.feetToMeters(11), 64.0);
    HUB_HOOD_ANGLE_MAP.put(Units.feetToMeters(12), 64.0);
    HUB_HOOD_ANGLE_MAP.put(Units.feetToMeters(13), 64.0);
    HUB_HOOD_ANGLE_MAP.put(Units.feetToMeters(14), 64.0);
    HUB_HOOD_ANGLE_MAP.put(Units.feetToMeters(15), 64.0);

    // Hub target: distance (m) → shooter RPM
    // Seed values from physics model, calibrated to 1800 RPM at 7ft
    HUB_RPM_MAP.put(Units.feetToMeters(3), 2692.0);
    HUB_RPM_MAP.put(Units.feetToMeters(4), 1843.0);
    HUB_RPM_MAP.put(Units.feetToMeters(5), 1748.0);
    HUB_RPM_MAP.put(Units.feetToMeters(6), 1758.0);
    HUB_RPM_MAP.put(Units.feetToMeters(7), 1800.0);
    HUB_RPM_MAP.put(Units.feetToMeters(8), 1855.0);
    HUB_RPM_MAP.put(Units.feetToMeters(9), 1916.0);
    HUB_RPM_MAP.put(Units.feetToMeters(10), 1979.0);
    HUB_RPM_MAP.put(Units.feetToMeters(11), 2043.0);
    HUB_RPM_MAP.put(Units.feetToMeters(12), 2107.0);
    HUB_RPM_MAP.put(Units.feetToMeters(13), 2168.0);
    HUB_RPM_MAP.put(Units.feetToMeters(14), 2230.0);
    HUB_RPM_MAP.put(Units.feetToMeters(15), 2290.0);

    // Pass target: distance (m) → hood angle (deg from horizontal)
    PASS_HOOD_ANGLE_MAP.put(Units.feetToMeters(10), 64.0);
    PASS_HOOD_ANGLE_MAP.put(Units.feetToMeters(15), 64.0);
    PASS_HOOD_ANGLE_MAP.put(Units.feetToMeters(20), 64.0);
    PASS_HOOD_ANGLE_MAP.put(Units.feetToMeters(25), 64.0);
    PASS_HOOD_ANGLE_MAP.put(Units.feetToMeters(30), 64.0);

    // Pass target: distance (m) → shooter RPM
    PASS_RPM_MAP.put(Units.feetToMeters(10), 1636.0);
    PASS_RPM_MAP.put(Units.feetToMeters(15), 2027.0);
    PASS_RPM_MAP.put(Units.feetToMeters(20), 2356.0);
    PASS_RPM_MAP.put(Units.feetToMeters(25), 2644.0);
    PASS_RPM_MAP.put(Units.feetToMeters(30), 2903.0);
  }

  // ===== PASS TARGET POSITIONS =====
  public static final Translation2d LOW_RED_PASS =
      new Translation2d(Meters.of(15.5), Meters.of(0.9));
  public static final Translation2d HIGH_RED_PASS =
      new Translation2d(Meters.of(15.5), Meters.of(7.1));
  public static final Translation2d LOW_BLUE_PASS =
      new Translation2d(Meters.of(1.1), Meters.of(0.9));
  public static final Translation2d HIGH_BLUE_PASS =
      new Translation2d(Meters.of(1.1), Meters.of(7.1));

  // ===== VELOCITY COMPENSATION =====
  // Minimum ball radial speed before solution is considered invalid (m/s)
  public static final double MIN_BALL_RADIAL_SPEED = 0.5;

  // ===== HIGH-FREQUENCY LOOP =====
  // Frequency for aiming computation and motor command threads (Hz)
  public static final double AIMING_FREQUENCY = 250.0;
}
