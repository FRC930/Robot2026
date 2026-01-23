package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOTalonFX implements IndexerIO {
  private TalonFX indexerMotor;

  private VoltageOut request;
  private Voltage setPoint = Volts.of(0);

  private final NeutralOut m_neutralOut = new NeutralOut();

  public IndexerIOTalonFX(int indexerMotorCAN, CANBus canbus) {
    indexerMotor = new TalonFX(indexerMotorCAN, canbus);
    request = new VoltageOut(0.0);
    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration configIndexer = new TalonFXConfiguration();
    configIndexer.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configIndexer.CurrentLimits.StatorCurrentLimit = 80.0;
    configIndexer.CurrentLimits.StatorCurrentLimitEnable = true;
    configIndexer.CurrentLimits.SupplyCurrentLimit = 10.0;
    configIndexer.CurrentLimits.SupplyCurrentLimitEnable = true;
    configIndexer.Voltage.PeakForwardVoltage = 16.0;
    configIndexer.Voltage.PeakReverseVoltage = 16.0;
    configIndexer.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    indexerMotor.getConfigurator().apply(configIndexer);
  }

  @Override
  public void setIndexerTarget(Voltage volts) {
    indexerMotor.setControl(request.withOutput(volts));
    setPoint = volts;
  }

  @Override
  public void stop() {
    indexerMotor.setControl(m_neutralOut);
    setPoint = Volts.zero();
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    inputs.angularVelocity.mut_replace(indexerMotor.getVelocity().getValue());
    inputs.voltage.mut_replace(indexerMotor.getMotorVoltage().getValue());
    inputs.setVoltage.mut_replace(setPoint);
  }
}
