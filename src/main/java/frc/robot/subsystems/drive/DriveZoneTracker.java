package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.util.VirtualSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveZoneTracker extends VirtualSubsystem implements DriveEvents {

  private final Supplier<Pose2d> poseSupplier;
  private boolean inNeutralZone = false;
  private boolean onUpperFieldHalf = false;

  private final Trigger inNeutralZoneTrigger = new Trigger(() -> inNeutralZone);
  private final Trigger onUpperFieldHalfTrigger = new Trigger(() -> onUpperFieldHalf);

  public DriveZoneTracker(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void periodic() {
    Pose2d pose = poseSupplier.get();
    double x = pose.getX();
    double y = pose.getY();

    inNeutralZone =
        x >= FieldConstants.LinesVertical.neutralZoneNear
            && x <= FieldConstants.LinesVertical.neutralZoneFar;
    onUpperFieldHalf = y > FieldConstants.LinesHorizontal.center;

    Logger.recordOutput("DriveZone/InNeutralZone", inNeutralZone);
    Logger.recordOutput("DriveZone/OnUpperFieldHalf", onUpperFieldHalf);
  }

  @Override
  public Trigger isInNeutralZone() {
    return inNeutralZoneTrigger;
  }

  @Override
  public Trigger isOnUpperFieldHalf() {
    return onUpperFieldHalfTrigger;
  }
}
