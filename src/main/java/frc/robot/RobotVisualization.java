package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

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

public class RobotVisualization extends VirtualSubsystem {
  private static RobotVisualization instance;

  private MutAngle turretTwist = Degrees.mutable(0);

  private MutAngle extenderTwist = Degrees.mutable(0);

  private final Mechanism2d primaryMechanism2d;

  private final MechanismRoot2d robotBaseRoot;
  private final MechanismLigament2d baseLigament2d =
      new MechanismLigament2d("RobotBase", 150, 0, 24, new Color8Bit(Color.kBlue));

  @Override
  public void periodic() {
    visualize();
  }

  private final String key;

  private RobotVisualization(String key) {
    super();
    this.key = key;
    primaryMechanism2d = new Mechanism2d(500, 300);

    robotBaseRoot = primaryMechanism2d.getRoot("2dBaseRoot", 225, 20);
    robotBaseRoot.append(baseLigament2d);
  }

  public Angle getTurretTwist() {
    return turretTwist;
  }

  public void setTurretTwist(Angle turretTwist) {
    this.turretTwist.mut_replace(turretTwist);
  }

  public void setTurretSource(MutAngle turretTwist) {
    this.turretTwist = turretTwist;
  }

  public Angle getExtenderTwist() {
    return extenderTwist;
  }

  public void setExtenderTwist(Angle extenderTwist) {
    this.extenderTwist.mut_replace(extenderTwist);
  }

  public void setExenderSource(MutAngle extenderTwist) {
    this.extenderTwist = extenderTwist;
  }

  public static RobotVisualization instance() {
    if (instance == null) {
      instance = new RobotVisualization("measured");
    }
    return instance;
  }

  private void visualize() {
    Pose3d turretPose =
        new Pose3d(TURRET_ATTACH_OFFSET.getTranslation(), TURRET_ATTACH_OFFSET.getRotation())
            .transformBy(
                new Transform3d(
                    new Translation3d(0, 0, 0),
                    new Rotation3d(0, this.turretTwist.in(Radians), 0)));

    Pose3d extenderPose =
        new Pose3d(EXTENDER_ATTACH_OFFSET.getTranslation(), EXTENDER_ATTACH_OFFSET.getRotation())
            .transformBy(
                new Transform3d(
                    new Translation3d(0, 0, 0),
                    new Rotation3d(-this.extenderTwist.in(Radians), 0, 0)));

    Logger.recordOutput("RobotState/Turret/" + key, turretPose);

    Logger.recordOutput("RobotState/Extender/" + key, extenderPose);
  }

  private static final Pose3d TURRET_ATTACH_OFFSET =
      new Pose3d(
          new Translation3d(Meters.of(0.092506), Inches.of(0), Inches.of(0)),
          new Rotation3d(Degrees.of(90), Degrees.of(0), Degrees.of(90)));

  private static final Pose3d EXTENDER_ATTACH_OFFSET =
      new Pose3d(
          new Translation3d(Meters.of(0.025107), Meters.of(0.), Meters.of(0.147639)),
          new Rotation3d(Degrees.of(90), Degrees.of(0), Degrees.of(90)));
}
