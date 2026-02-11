package frc.robot.subsystems.hood;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import frc.robot.util.Gains;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodInputs {
    public MutAngle hoodAngle;
    public MutAngle hoodSetAngle;
  }

  public default void setHoodTarget(Angle target) {}
  ;

  public default void stop() {}
  ;

  public default void updateInputs(HoodInputs input) {}
  ;

  public default void setGains(Gains gains) {}
  ;
}
