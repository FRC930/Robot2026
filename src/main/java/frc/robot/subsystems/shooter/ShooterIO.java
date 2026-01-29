package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import frc.robot.util.Gains;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterInputs {
    // inputs for shooter
    public MutAngularVelocity shooterAngularVelocity;
    public MutAngularVelocity shooterSetpoint;
  }

  public void setShooterTarget(AngularVelocity target);

  public void stop();

  public void updateInputs(ShooterInputs input);

  public void setGains(Gains gains);
}
