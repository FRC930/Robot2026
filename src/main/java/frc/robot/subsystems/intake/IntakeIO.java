package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeInputs {
    public MutAngularVelocity intakeAngularVelocity;
    public MutVoltage intakeVoltage;
    public MutVoltage intakeSetVoltage;
    public MutAngle intakeExtenderAngle;
  }

  public void setIntakerTarget(Voltage target);

  public void setIntakerExtenderTarget(Angle angle);

  public void stop();

  public void updateInputs(IntakeInputs input);
}
