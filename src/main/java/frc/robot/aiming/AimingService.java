package frc.robot.aiming;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.PoseSnapshot;
import frc.robot.util.EnumState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Central aiming service that computes hood angle, and shooter RPM. Computation runs at 250Hz via
 * AimingThread; logging and visualization run at 50Hz via periodic().
 */
public class AimingService extends VirtualSubsystem implements AimingEvents {

  private final Supplier<PoseSnapshot> snapshotSupplier;

  private final EnumState<AimingTarget> targetState =
      new EnumState<>("Aiming/Target", AimingTarget.HUB);

  // Thread-safe mirror of targetState (EnumState.currentState is not volatile)
  private volatile AimingTarget currentTarget = AimingTarget.HUB;

  // Volatile outputs written by computeAimingSolution() at 250Hz
  private volatile double hoodAngleDeg = 45.0;
  private volatile double shooterRPM = 0.0;
  private volatile double distanceToTargetM = 0.0;
  private volatile boolean solutionValid = false;

  // Cached launcher params for BallTrajectorySim at 50Hz
  private volatile double cachedLauncherSpeed = 0.0;
  private volatile double cachedLauncherAngleRad = 0.0;

  // EMA smoothing — alpha in (0, 1]. Lower = smoother, 1.0 = no filtering.
  private static final LoggedTunableNumber rpmEmaAlpha =
      new LoggedTunableNumber("Aiming/Smoothing/rpmEmaAlpha", 0.08);
  private static final LoggedTunableNumber hoodEmaAlpha =
      new LoggedTunableNumber("Aiming/Smoothing/hoodEmaAlpha", 0.10);

  // EMA state (only written by AimingThread, no synchronization needed)
  private double smoothedRPM = 0.0;
  private double smoothedHoodDeg = 0.0;
  private boolean emaInitialized = false;
  private boolean usingFarAngle = false;

  public static final BallTrajectorySim trajectorySim = new BallTrajectorySim();

  public AimingService(Supplier<PoseSnapshot> snapshotSupplier) {
    this.snapshotSupplier = snapshotSupplier;
  }

  public void setTarget(AimingTarget target) {
    targetState.set(target);
    currentTarget = target;
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

  /** Called at 250Hz by AimingThread. Reads snapshot, extrapolates pose, computes solution. */
  public void computeAimingSolution() {
    PoseSnapshot snapshot = snapshotSupplier.get();

    // Linear extrapolation: estimate current pose using velocity * dt
    double now = Timer.getFPGATimestamp();
    double dt = MathUtil.clamp(now - snapshot.timestampSeconds(), 0.0, 0.1);

    ChassisSpeeds speeds = snapshot.chassisSpeeds();
    Rotation2d heading = snapshot.heading();

    // Field-relative velocity for extrapolation
    double cos = heading.getCos();
    double sin = heading.getSin();
    double vxField = speeds.vxMetersPerSecond * cos - speeds.vyMetersPerSecond * sin;
    double vyField = speeds.vxMetersPerSecond * sin + speeds.vyMetersPerSecond * cos;

    Pose2d basePose = snapshot.pose();
    Pose2d robotPose =
        new Pose2d(
            basePose.getX() + vxField * dt,
            basePose.getY() + vyField * dt,
            basePose.getRotation().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt)));
    Rotation2d extrapolatedHeading = robotPose.getRotation();

    // Compute the aiming solution using extrapolated pose
    computeForPose(robotPose, speeds, extrapolatedHeading);
  }

  /** 50Hz logging and visualization only. Reads volatile fields. */
  @Override
  public void periodic() {
    targetState.log();
    logOutputs();

    // Trajectory visualization at 50Hz — skip on real robot to save CPU
    if (Constants.currentMode != Constants.Mode.REAL) {
      PoseSnapshot snapshot = snapshotSupplier.get();
      if (solutionValid) {
        Rotation2d heading = snapshot.heading();
        Pose2d robotPos = RobotContainer.driveSimulation.getSimulatedDriveTrainPose();
        Translation2d translation = robotPos.getTranslation();
        trajectorySim.simulate(translation, cachedLauncherAngleRad, cachedLauncherSpeed);
      } else {
        trajectorySim.publishEmpty();
      }
    }
  }

  private void computeForPose(Pose2d robotPose, ChassisSpeeds speeds, Rotation2d heading) {
    Translation3d target = getTargetPosition();

    // Compute distances to target
    double dx = target.getX();
    double dy = target.getY();
    double horizontalDistance = Math.hypot(dx, dy);
    double verticalDistance = target.getZ();

    distanceToTargetM = horizontalDistance;

    // Field-frame angle to target, converted to turret-frame (0° = robot backward)
    double fieldAngleToTargetRad = Math.atan2(dy, dx);

    // Velocity compensation: project turret velocity onto radial and tangential axes
    double ux = Math.cos(fieldAngleToTargetRad);
    double uy = Math.sin(fieldAngleToTargetRad);
    double vRadial = ux + uy;
    double vTangential = -uy + ux;

    double launchAngleDeg;
    if (currentTarget == AimingTarget.HUB) {
      double threshold = AimingConstants.FAR_DISTANCE_THRESHOLD_M.get();
      double hysteresis = AimingConstants.FAR_DISTANCE_HYSTERESIS_M.get();
      if (usingFarAngle) {
        usingFarAngle = horizontalDistance > (threshold - hysteresis);
      } else {
        usingFarAngle = horizontalDistance > (threshold + hysteresis);
      }
      launchAngleDeg =
          usingFarAngle
              ? AimingConstants.TARGET_FAR_LAUNCH_ANGLE_DEG.get()
              : AimingConstants.TARGET_LAUNCH_ANGLE_DEG.get();
    } else {
      launchAngleDeg = AimingConstants.TARGET_PASS_LAUNCH_ANGLE_DEG.get();
    }
    double fieldLaunchAngle = Math.toRadians(launchAngleDeg);
    double tanTheta = Math.tan(fieldLaunchAngle);
    double cosTheta = Math.cos(fieldLaunchAngle);
    double denominator = horizontalDistance * tanTheta - verticalDistance;

    if (denominator <= 0) {
      solutionValid = false;
      return;
    }

    double fieldTotalSpeed =
        (horizontalDistance / cosTheta) * Math.sqrt(AimingConstants.GRAVITY / (2.0 * denominator));

    double fieldHorizontal = fieldTotalSpeed * Math.cos(fieldLaunchAngle);
    double fieldVertical = fieldTotalSpeed * Math.sin(fieldLaunchAngle);

    double launcherRadial = fieldHorizontal - vRadial;
    double launcherTangential = -vTangential;

    if (launcherRadial <= AimingConstants.MIN_BALL_RADIAL_SPEED) {
      solutionValid = false;
      return;
    }

    double launcherHorizontal = Math.hypot(launcherRadial, launcherTangential);
    double launcherSpeed = Math.hypot(launcherHorizontal, fieldVertical);
    double launcherAngleRad = Math.atan2(fieldVertical, launcherHorizontal);

    // Yaw correction: aim off-target to cancel tangential velocity
    double yawCorrectionRad = Math.atan2(-vTangential, launcherRadial);
    // Convert launcher speed to RPM
    double shooterSurfaceSpeedMps = launcherSpeed / AimingConstants.SPEED_TRANSFER_RATIO.get();
    double flywheelCircumference = 2.0 * Math.PI * AimingConstants.FLYWHEEL_RADIUS_M.get();
    double rpm = (shooterSurfaceSpeedMps / flywheelCircumference) * 60.0;

    // Distance-based RPM scaling to compensate for unmodeled drag
    double distanceOffset = horizontalDistance - AimingConstants.RPM_REF_DISTANCE_M.get();
    rpm *= 1.0 + AimingConstants.RPM_DISTANCE_SCALE.get() * distanceOffset;

    // Clamp and validate
    double launcherAngleDeg = Math.toDegrees(launcherAngleRad);
    boolean hoodInRange =
        launcherAngleDeg >= AimingConstants.HOOD_MIN_DEG
            && launcherAngleDeg <= AimingConstants.HOOD_MAX_DEG;
    boolean rpmInRange =
        rpm >= AimingConstants.SHOOTER_MIN_RPM && rpm <= AimingConstants.SHOOTER_MAX_RPM;

    solutionValid = hoodInRange && rpmInRange;

    double clampedHood =
        MathUtil.clamp(
            launcherAngleDeg, AimingConstants.HOOD_MIN_DEG, AimingConstants.HOOD_MAX_DEG);
    double clampedRPM =
        MathUtil.clamp(rpm, AimingConstants.SHOOTER_MIN_RPM, AimingConstants.SHOOTER_MAX_RPM);

    // EMA smoothing to reduce high-frequency noise from pose estimation
    if (!emaInitialized) {
      smoothedHoodDeg = clampedHood;
      smoothedRPM = clampedRPM;
      emaInitialized = true;
    } else {
      double aH = hoodEmaAlpha.get();
      double aR = rpmEmaAlpha.get();
      smoothedHoodDeg = aH * clampedHood + (1.0 - aH) * smoothedHoodDeg;
      smoothedRPM = aR * clampedRPM + (1.0 - aR) * smoothedRPM;
    }

    hoodAngleDeg = smoothedHoodDeg;
    shooterRPM = smoothedRPM;

    // Cache launcher params for 50Hz trajectory visualization
    cachedLauncherSpeed = launcherSpeed;
    cachedLauncherAngleRad = launcherAngleRad;
  }

  private Translation3d getTargetPosition() {
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    switch (currentTarget) {
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

  private void logOutputs() {
    Logger.recordOutput("Aiming/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("Aiming/ShooterRPM", shooterRPM);
    Logger.recordOutput("Aiming/DistanceToTargetM", distanceToTargetM);
    Logger.recordOutput("Aiming/SolutionValid", solutionValid);
    PoseSnapshot snapshot = snapshotSupplier.get();
    Logger.recordOutput("Aiming/RobotPose", snapshot.pose());
    Logger.recordOutput("Aiming/TargetPosition", getTargetPosition());
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
