package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Immutable snapshot of drive state, published by Drive.periodic() at 50Hz and consumed by the
 * aiming thread at 250Hz. Since records are immutable and the reference is volatile, reading is
 * always consistent without locks.
 */
public record PoseSnapshot(
    Pose2d pose, ChassisSpeeds chassisSpeeds, Rotation2d heading, double timestampSeconds) {

  public static final PoseSnapshot IDENTITY =
      new PoseSnapshot(new Pose2d(), new ChassisSpeeds(), Rotation2d.kZero, 0.0);
}
