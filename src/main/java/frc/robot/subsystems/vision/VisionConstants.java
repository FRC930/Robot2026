// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String frontLeftCamera = "limelight-frontl";
  public static String frontRightCamera = "limelight-frontr";
  public static String frontLeftForwardCamera = "limelight-frontlf";
  public static String backRightCamera = "limelight-backr";
  public static String questCamName = "Quest";
  /*
   * CAMERA POSITIONS ON ROBOT (looking down from above, with front of robot at top of page)
   *
   * Front Left (x,y)  Front Right (x,y)
   * +/+               +/-
   *
   * Back Left (x,y)   Back Right (x,y)
   * -/+               -/-
   *
   * NOTE: Limelight right (LL Right) is reversed in code vs on the actual limelight (if its - in code, its + on limelight, and vice versa)
   * NOTE: Limelight pitch (LL Pitch) is reversed in code vs on the actual limelight (if its - in code, its + on limelight, and vice versa)
   */

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  // front cam values on robot,
  // LL Forward 0.391, LL Right 0.0, LL up 0.198, LL Roll 0.0, LL Pitch 0.0, LL Yaw 0.0.

  public static Transform3d robotToFrontLeftCamera =
      new Transform3d(
          Units.inchesToMeters(9.29),
          Units.inchesToMeters(15.96),
          Units.inchesToMeters(23.34),
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(90.0)));

  public static Transform3d robotToFrontRightCamera =
      new Transform3d(
          Units.inchesToMeters(9.29),
          Units.inchesToMeters(-15.96),
          Units.inchesToMeters(23.34),
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(-90)));

  public static Transform3d robotToFrontLeftForwardCamera =
      new Transform3d(
          Units.inchesToMeters(12.52),
          Units.inchesToMeters(13.0),
          Units.inchesToMeters(23.34),
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(0)));

  public static Transform3d robotToBackRightCamera =
      new Transform3d(
          Units.inchesToMeters(-12.89),
          Units.inchesToMeters(8.5),
          Units.inchesToMeters(23.34),
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0, // Camera 2
        1.0 // Camera 3
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
