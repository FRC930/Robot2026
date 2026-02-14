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

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera1Name = "limelight-one";
  public static String camera2Name = "limelight-two";
  public static String camera3Name = "limelight-three";
  public static String camera4Name = "limelight-four";
  public static String questCamName = "Quest";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  // front cam values on robot,
  // LL Forward 0.391, LL Right 0.0, LL up 0.198, LL Roll 0.0, LL Pitch 0.0, LL Yaw 0.0.
  //left 90
  public static Transform3d robotToCamera1 =
      new Transform3d(
        -15.96 ,9.29,23.34, new Rotation3d(0.0, Math.toRadians(0.0), Math.toDegrees(-90.0)));

        //right 90
  public static Transform3d robotToCamera2 =
      new Transform3d(15.96, 9.29, 23.34, new Rotation3d(0.0, -0.4, Math.toDegrees(90)));

      // front0
    public static Transform3d robotToCamera3 =
      new Transform3d(13.0, 12.52, 23.34, new Rotation3d(0.0, -0.4, Math.toDegrees(0)));
//back 180
    public static Transform3d robotToCamera4 =
      new Transform3d(-8.5, -12.89, 23.34, new Rotation3d(0.0, -0.4, Math.toDegrees(180)));

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
        1.0,
        1.0
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
