package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.util.Gains;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterInputs {
    // inputs for shooter
    public MutAngularVelocity shooterAngularVelocity;
    public MutAngularVelocity shooterSetpoint;
    public MutCurrent shooterTorqueCurrent;
    public MutVoltage shooterVoltage;
    public MutCurrent shooterSupplyCurrent;
  }

  public default void setShooterTarget(AngularVelocity target) {}
  ;

  public default void stop() {}
  ;

  public default void updateInputs(ShooterInputs input) {}
  ;

  public default void setGains(Gains gains) {}
  ;

  public default void setSupplyCurrentLimit(double amps) {}
  ;
}
