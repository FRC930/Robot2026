package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.util.Gains;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public class IndexerInputs {
    public MutAngularVelocity indexerVelocity;
    public MutAngularVelocity indexerSetPoint;
    public MutVoltage indexerVoltage;
    public MutVoltage indexerVoltageSetPoint;
    public MutCurrent indexerSupplyCurrent;
    public MutCurrent indexerTorqueCurrent;

    public MutAngularVelocity feederVelocity;
    public MutAngularVelocity feederSetPoint;
    public MutVoltage feederVoltage;
    public MutVoltage feederVoltageSetPoint;
    public MutCurrent feederSupplyCurrent;
    public MutCurrent feederTorqueCurrent;
  }

  public default void setIndexerTarget(AngularVelocity velocity) {}
  ;

  public default void setFeederTarget(AngularVelocity velocity) {}
  ;

  public default void stop() {}
  ;

  public default void updateInputs(IndexerInputs inputs) {}
  ;
  
  public default void setIndexerGains(Gains gains) {}
  ;

  public default void setFeederGains(Gains gains) {}
  ;
}
