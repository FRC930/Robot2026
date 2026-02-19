package frc.robot.aiming;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.util.EnumState;
import frc.robot.util.VirtualSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Central aiming service that computes turret angle, hood angle, and shooter RPM every cycle.
 * Performs velocity-space compensation to account for robot motion while shooting.
 */
public class AimingService extends VirtualSubsystem implements AimingEvents {

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
  private final Supplier<Rotation2d> robotHeadingSupplier;

  private final EnumState<AimingTarget> targetState =
      new EnumState<>("Aiming/Target", AimingTarget.HUB);

  private double turretAngleDeg = 0.0;
  private double hoodAngleDeg = 45.0;
  private double shooterRPM = 0.0;
  private double distanceToTargetM = 0.0;
  private boolean solutionValid = false;

  private final BallTrajectorySim trajectorySim = new BallTrajectorySim();

  public AimingService(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Supplier<Rotation2d> robotHeadingSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.robotHeadingSupplier = robotHeadingSupplier;
  }

  public void setTarget(AimingTarget target) {
    targetState.set(target);
  }

  @Override
  public Trigger isAimingAtHub() {
    return targetState.is(AimingTarget.HUB);
  }

  @Override
  public Trigger isAimingAtPassLow() {
    return targetState.is(AimingTarget.PASS_LOW);
  }

  @Override
  public Trigger isAimingAtPassHigh() {
    return targetState.is(AimingTarget.PASS_HIGH);
  }

  @Override
  public void periodic() {
    Pose2d robotPose = robotPoseSupplier.get();
    ChassisSpeeds speeds = chassisSpeedsSupplier.get();
    Rotation2d heading = robotHeadingSupplier.get();

    Translation3d target = getTargetPosition();
    Translation2d turretFieldPos = computeTurretFieldPosition(robotPose, heading);

    // Compute distances to target
    double dx = target.getX() - turretFieldPos.getX();
    double dy = target.getY() - turretFieldPos.getY();
    double horizontalDistance = Math.hypot(dx, dy);
    double verticalDistance = target.getZ() - AimingConstants.TURRET_PIVOT_HEIGHT_METERS;

    distanceToTargetM = horizontalDistance;

    // Field-frame angle to target, converted to turret-frame (0° = robot backward)
    double fieldAngleToTargetRad = Math.atan2(dy, dx);
    double turretAngleRad =
        MathUtil.angleModulus(fieldAngleToTargetRad - heading.getRadians() - Math.PI);
    double rawTurretAngleDeg = Math.toDegrees(turretAngleRad);

    // Velocity compensation: project turret velocity onto radial and tangential axes
    Translation2d turretVelocity = computeTurretVelocity(speeds, heading);
    double ux = Math.cos(fieldAngleToTargetRad);
    double uy = Math.sin(fieldAngleToTargetRad);
    double vRadial = turretVelocity.getX() * ux + turretVelocity.getY() * uy;
    double vTangential = -turretVelocity.getX() * uy + turretVelocity.getY() * ux;

    // Compute field-frame trajectory: pick a desired launch angle, compute the exact speed
    // needed to hit the target at that angle. Ball arrives descending for angles above ~45 deg.
    // Formula: v = (x / cos(theta)) * sqrt(g / (2 * (x*tan(theta) - y)))
    double fieldLaunchAngle = Math.toRadians(AimingConstants.TARGET_LAUNCH_ANGLE_DEG.get());
    double tanTheta = Math.tan(fieldLaunchAngle);
    double cosTheta = Math.cos(fieldLaunchAngle);
    double denominator = horizontalDistance * tanTheta - verticalDistance;

    if (denominator <= 0) {
      // Target is too close or angle too shallow to reach the height
      solutionValid = false;
      logOutputs();
      trajectorySim.publishEmpty();
      return;
    }

    double fieldTotalSpeed =
        (horizontalDistance / cosTheta) * Math.sqrt(AimingConstants.GRAVITY / (2.0 * denominator));

    // Compute launcher exit velocity (compensate for both radial and tangential turret velocity).
    // The launcher must provide a horizontal velocity that, combined with the turret's velocity,
    // gives exactly fieldHorizontal toward the target and zero tangential drift.
    double fieldHorizontal = fieldTotalSpeed * Math.cos(fieldLaunchAngle);
    double fieldVertical = fieldTotalSpeed * Math.sin(fieldLaunchAngle);

    // Solve the two constraints simultaneously:
    //   launcherH * cos(yawCorr) + vRadial     = fieldHorizontal  (radial)
    //   launcherH * sin(yawCorr) + vTangential  = 0               (tangential)
    double launcherRadial = fieldHorizontal - vRadial;
    double launcherTangential = -vTangential;

    if (launcherRadial <= AimingConstants.MIN_BALL_RADIAL_SPEED) {
      solutionValid = false;
      logOutputs();
      trajectorySim.publishEmpty();
      return;
    }

    double launcherHorizontal = Math.hypot(launcherRadial, launcherTangential);
    double launcherSpeed = Math.hypot(launcherHorizontal, fieldVertical);
    double launcherAngleRad = Math.atan2(fieldVertical, launcherHorizontal);

    // Yaw correction: aim off-target to cancel tangential velocity
    double yawCorrectionRad = Math.atan2(-vTangential, launcherRadial);
    double compensatedTurretDeg = rawTurretAngleDeg + Math.toDegrees(yawCorrectionRad);

    // Convert launcher speed to RPM
    double shooterSurfaceSpeedMps = launcherSpeed / AimingConstants.SPEED_TRANSFER_RATIO.get();
    double flywheelCircumference = 2.0 * Math.PI * AimingConstants.FLYWHEEL_RADIUS_M.get();
    double rpm = (shooterSurfaceSpeedMps / flywheelCircumference) * 60.0;

    // Clamp and validate
    boolean turretInRange =
        compensatedTurretDeg >= AimingConstants.TURRET_MIN_DEG
            && compensatedTurretDeg <= AimingConstants.TURRET_MAX_DEG;
    double launcherAngleDeg = Math.toDegrees(launcherAngleRad);
    boolean hoodInRange =
        launcherAngleDeg >= AimingConstants.HOOD_MIN_DEG
            && launcherAngleDeg <= AimingConstants.HOOD_MAX_DEG;
    boolean rpmInRange =
        rpm >= AimingConstants.SHOOTER_MIN_RPM && rpm <= AimingConstants.SHOOTER_MAX_RPM;

    solutionValid = turretInRange && hoodInRange && rpmInRange;

    turretAngleDeg =
        MathUtil.clamp(
            compensatedTurretDeg, AimingConstants.TURRET_MIN_DEG, AimingConstants.TURRET_MAX_DEG);
    hoodAngleDeg =
        MathUtil.clamp(
            launcherAngleDeg, AimingConstants.HOOD_MIN_DEG, AimingConstants.HOOD_MAX_DEG);
    shooterRPM =
        MathUtil.clamp(rpm, AimingConstants.SHOOTER_MIN_RPM, AimingConstants.SHOOTER_MAX_RPM);

    // Trajectory visualization (launcher exit speed + turret velocity = field trajectory)
    double fieldTurretYaw = heading.getRadians() + Math.toRadians(turretAngleDeg);
    trajectorySim.simulate(
        turretFieldPos, fieldTurretYaw, launcherAngleRad, launcherSpeed, turretVelocity);

    logOutputs();
  }

  /**
   * Get the correct target position based on aiming target state and alliance color. For HUB,
   * targets the top rim so the ball arcs over and drops in. Pass targets are at ground level.
   */
  private Translation3d getTargetPosition() {
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    switch (targetState.get()) {
      case PASS_LOW:
        Translation2d lowPass =
            isRed ? AimingConstants.LOW_RED_PASS : AimingConstants.LOW_BLUE_PASS;
        return new Translation3d(lowPass);
      case PASS_HIGH:
        Translation2d highPass =
            isRed ? AimingConstants.HIGH_RED_PASS : AimingConstants.HIGH_BLUE_PASS;
        return new Translation3d(highPass);
      case HUB:
      default:
        return isRed ? FieldConstants.Hub.oppTopCenterPoint : FieldConstants.Hub.topCenterPoint;
    }
  }

  /** Compute the turret pivot position in field coordinates. */
  private Translation2d computeTurretFieldPosition(Pose2d robotPose, Rotation2d heading) {
    Translation2d rotatedOffset = AimingConstants.TURRET_OFFSET_FROM_CENTER.rotateBy(heading);
    return robotPose.getTranslation().plus(rotatedOffset);
  }

  /**
   * Compute the velocity of the turret pivot in field-frame. turretVelocity =
   * robotLinearVelocity_field + omega x turretOffset_field
   */
  private Translation2d computeTurretVelocity(ChassisSpeeds speeds, Rotation2d heading) {
    // Convert robot-relative chassis speeds to field-relative
    double cos = heading.getCos();
    double sin = heading.getSin();
    double vxField = speeds.vxMetersPerSecond * cos - speeds.vyMetersPerSecond * sin;
    double vyField = speeds.vxMetersPerSecond * sin + speeds.vyMetersPerSecond * cos;

    // Rotational contribution at turret offset: omega x r (2D cross product)
    Translation2d rotatedOffset = AimingConstants.TURRET_OFFSET_FROM_CENTER.rotateBy(heading);
    double omega = speeds.omegaRadiansPerSecond;
    double vxRot = -omega * rotatedOffset.getY();
    double vyRot = omega * rotatedOffset.getX();

    return new Translation2d(vxField + vxRot, vyField + vyRot);
  }

  private void logOutputs() {
    Logger.recordOutput("Aiming/TurretAngleDeg", turretAngleDeg);
    Logger.recordOutput("Aiming/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("Aiming/ShooterRPM", shooterRPM);
    Logger.recordOutput("Aiming/DistanceToTargetM", distanceToTargetM);
    Logger.recordOutput("Aiming/SolutionValid", solutionValid);
    Logger.recordOutput("Aiming/RobotPose", robotPoseSupplier.get());
    Logger.recordOutput("Aiming/TargetPosition", getTargetPosition());
  }

  public double getTurretAngleDeg() {
    return turretAngleDeg;
  }

  public double getHoodAngleDeg() {
    return hoodAngleDeg;
  }

  public double getShooterRPM() {
    return shooterRPM;
  }

  public boolean isSolutionValid() {
    return solutionValid;
  }

  public double getDistanceToTargetM() {
    return distanceToTargetM;
  }
}
