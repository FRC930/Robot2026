package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public class IndexerInputs {
    public MutAngularVelocity angularVelocity;
    public MutVoltage voltage;
    public MutVoltage setVoltage;
  }

  public void setIndexerTarget(Voltage volts);

  public void stop();

  public void updateInputs(IndexerInputs inputs);
}
