package frc.robot.subsystems.compactor;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.util.Gains;
import org.littletonrobotics.junction.AutoLog;

public interface CompactorIO {
  @AutoLog
  public static class CompactorInputs {
    public MutDistance distance;
    public MutLinearVelocity velocity;
    public MutDistance setPoint;
    public MutVoltage voltage;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public default void setCompactorHeight(Distance target) {}
  ;

  public default void setCompactorVelocity(AngularVelocity veloctiy) {}
  ;

  public default void stop() {}
  ;

  public default void updateInputs(CompactorInputs input) {}
  ;

  public default void setGains(Gains gains) {}
  ;
}
