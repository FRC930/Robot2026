package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class RobotState extends VirtualSubsystem {
  private static RobotState instance;

  private MutAngle wristTwist = Degrees.mutable(0);

  private final Mechanism2d primaryMechanism2d;
  private final MechanismRoot2d primaryMechanismRoot;

  private final MechanismRoot2d wristMechanismRoot;
  private final MechanismLigament2d wristMechanismLigament;

  private final MechanismRoot2d robotBaseRoot;
  private final MechanismLigament2d baseLigament2d =
      new MechanismLigament2d("RobotBase", 150, 0, 24, new Color8Bit(Color.kBlue));

  @Override
  public void periodic() {
    visualize();
  }

  private final String key;

  private RobotState(String key) {
    super();
    this.key = key;
    primaryMechanism2d = new Mechanism2d(500, 300);
    wristMechanismLigament =
        new MechanismLigament2d(
            "WristLigament",
            Centimeters.convertFrom(18, Inches),
            wristTwist.in(Degrees),
            5,
            new Color8Bit(Color.kOrange));

    robotBaseRoot = primaryMechanism2d.getRoot("2dBaseRoot", 225, 20);
    robotBaseRoot.append(baseLigament2d);

    primaryMechanismRoot = primaryMechanism2d.getRoot("2dPrimary", 300, 20);

    wristMechanismRoot = primaryMechanism2d.getRoot("2dWrist", 30, 20);
    wristMechanismRoot.append(wristMechanismLigament);

    // SmartDashboard.putData("Mech2d",primaryMechanism2d);

  }

  public Angle getWristTwist() { // 67
    return wristTwist;
  }

  public void setWristTwist(Angle wristTwist) {
    this.wristTwist.mut_replace(wristTwist);
  }

  public void setWristSource(MutAngle wristTwist) {
    this.wristTwist = wristTwist;
  }

  public static RobotState instance() {
    if (instance == null) {
      instance = new RobotState("measured");
    }
    return instance;
  }

  private void visualize() {
    Pose3d wristPose =
        new Pose3d(TURRET_ATTACH_OFFSET.getTranslation(), TURRET_ATTACH_OFFSET.getRotation())
            .transformBy(new Transform3d(new Translation3d(0, 0, 0), new Rotation3d()));

    wristMechanismLigament.setAngle(wristTwist.in(Degrees));

    Logger.recordOutput("RobotState/Wrist/" + key, wristPose);
  }

  private static final Pose3d TURRET_ATTACH_OFFSET =
      new Pose3d(
          new Translation3d(Inches.of(0), Inches.of(0.), Inches.of(0)),
          new Rotation3d(Degrees.of(90.), Degrees.of(0), Degrees.of(0)));
}
