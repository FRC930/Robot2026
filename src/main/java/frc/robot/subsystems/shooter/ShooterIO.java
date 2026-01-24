package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterInputs {
    //inputs for shooter
    public MutAngularVelocity shooterAngularVelocity;
    public MutVoltage shooterVoltage;
    public MutVoltage shooterSetVoltage;
  }

  public void setShooterTarget(Voltage target);

  public void stop();

  public void updateInputs(ShooterInputs input);
}
