package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Fahrenheit;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.util.Gains;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeInputs {
    public MutAngularVelocity rollerVelocity;
    public MutAngularVelocity rollerVelocitySetPoint;
    public MutCurrent rollerSupplyCurrent;
    public MutCurrent rollerTorqueCurrent;
    public MutVoltage rollerVoltage;
    public MutTemperature leaderRollerTemp;
    public MutTemperature followerRollerTemp = Fahrenheit.mutable(0);

    public MutVoltage extenderVoltage;
    public MutAngle extenderAngle;
    public MutAngle extenderAngleSetPoint;
    public MutCurrent extenderSupplyCurrent;
    public MutCurrent extenderTorqueCurrent;
    // Emulated Angle/SetAngle (given extender using current limits)
    // public MutAngle extenderEmulatedAngle;
    // public MutAngle extenderEmulatedSetAngle;
  }

  public default void setRollerTargetSpeed(AngularVelocity target) {}
  ;

  public default void setExtenderTargetAngle(Angle angle) {}
  ;

  public default void stop() {}
  ;

  public default void updateInputs(IntakeInputs input) {}
  ;

  public default void setRollerGains(Gains gains) {}
  ;

  public default void setExtenderGains(Gains gains) {}
  ;
}
