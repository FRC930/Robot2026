package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public class IndexerInputs {
    public MutCurrent indexerSupplyCurrent;
    public MutAngularVelocity indexerAngularVelocity;
    public MutAngularVelocity indexerSetpoint;

    public MutCurrent feederSupplyCurrent;
    public MutVoltage feederVoltage;
    public MutVoltage feederSetVoltage;
  }

  public default void setIndexerTarget(AngularVelocity velocity) {}
  ;

  public default void setFeederTarget(Voltage volts) {}
  ;

  public default void stop() {}
  ;

  public default void updateInputs(IndexerInputs inputs) {}
  ;
}
