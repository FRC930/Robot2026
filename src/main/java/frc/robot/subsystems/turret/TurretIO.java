package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.util.Gains;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretInputs {
    // inputs for turret
    public MutAngularVelocity turretAngularVelocity;
    public MutAngle turretAngle;
    public MutAngle turretSetAngle;
    public MutAngle canCoderAngle1;
    public MutAngle canCoderAngle2;
    public MutVoltage turretVoltage;
    public MutCurrent turretSupplyCurrent;
    public MutCurrent turretTorqueCurrent;
  }

  public default void setTarget(double angle) {}
  ;

  public default void stop() {}
  ;

  public default void updateInputs(TurretInputs input) {}
  ;

  public default void setGains(Gains gains) {}
  ;

  public default Angle getCanCoderAngle1() {
    return Degrees.of(0.0);
  }
  ;

  public default Angle getCanCoderAngle2() {
    return Degrees.of(0.0);
  }
  ;
}
