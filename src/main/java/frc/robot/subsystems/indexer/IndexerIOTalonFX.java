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
  private TalonFX feederMotor;

  private VoltageOut requestIndexer;
  private VoltageOut requestFeeder;
  private Voltage indexerSetPoint = Volts.of(0);
  private Voltage feederSetPoint = Volts.of(0);

  private final NeutralOut m_neutralOut = new NeutralOut();

  public IndexerIOTalonFX(int indexerMotorCAN, int feederMotorCAN, CANBus canbus) {
    indexerMotor = new TalonFX(indexerMotorCAN, canbus);
    feederMotor = new TalonFX(feederMotorCAN, canbus);
    requestIndexer = new VoltageOut(0.0); //TODO: Switch to Velocity Controller with PIDs
    requestFeeder = new VoltageOut(0.0);
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
    if (!(volts.in(Volts) == indexerSetPoint.in(Volts))) {
      indexerMotor.setControl(requestIndexer.withOutput(volts));
      indexerSetPoint = volts;
    }
  }

  @Override
  public void setFeederTarget(Voltage volts) {
    if (volts.in(Volts) != feederSetPoint.in(Volts)) {
      feederMotor.setControl(requestFeeder.withOutput(volts));
      feederSetPoint = volts;
    }
  }

  @Override
  public void stop() {
    indexerMotor.setControl(m_neutralOut);
    feederMotor.setControl(m_neutralOut);
    indexerSetPoint = Volts.zero();
    feederSetPoint = Volts.zero();
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    inputs.indexerSupplyCurrent.mut_replace(indexerMotor.getSupplyCurrent().getValue());
    inputs.indexerVoltage.mut_replace(indexerMotor.getMotorVoltage().getValue());
    inputs.indexerSetVoltage.mut_replace(indexerSetPoint);

    inputs.feederSupplyCurrent.mut_replace(feederMotor.getSupplyCurrent().getValue());
    inputs.feederVoltage.mut_replace(feederMotor.getMotorVoltage().getValue());
    inputs.feederSetVoltage.mut_replace(feederSetPoint);
  }
}
