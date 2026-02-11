package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.Gains;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeInputs {
    public MutAngularVelocity rollerVelocity;
    public MutAngularVelocity rollerVelocitySetPoint;
    public MutCurrent rollerSupplyCurrent;

    public MutVoltage extenderVoltage;
    public MutVoltage extenderVoltageSetPoint;
    public MutCurrent extenderSupplyCurrent;
    // Emulated Angle/SetAngle (given extender using current limits)
    public MutAngle extenderEmulatedAngle;
    public MutAngle extenderEmulatedSetAngle;
  }

  public default void setRollerTargetSpeed(AngularVelocity target) {}
  ;

  public default void setExtenderTargetVolts(Voltage voltage) {}
  ;

  public default void stop() {}
  ;

  public default void updateInputs(IntakeInputs input) {}
  ;

  public default void setGains(Gains gains) {}
  ;
}
