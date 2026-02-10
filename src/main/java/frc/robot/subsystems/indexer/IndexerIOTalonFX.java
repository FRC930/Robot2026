package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOTalonFX implements IndexerIO {
  private TalonFX indexerMotor;
  private TalonFX feederMotor;

  private MotionMagicVelocityTorqueCurrentFOC requestIndexer;
  private VoltageOut requestFeeder;
  private AngularVelocity indexerSetPoint = RPM.of(0);
  private Voltage feederSetPoint = Volts.of(0);

  private final NeutralOut m_neutralOut = new NeutralOut();

  public IndexerIOTalonFX(int indexerMotorCAN, int feederMotorCAN, CANBus canbus) {
    indexerMotor = new TalonFX(indexerMotorCAN, canbus);
    feederMotor = new TalonFX(feederMotorCAN, canbus);
    requestIndexer = new MotionMagicVelocityTorqueCurrentFOC(RPM.of(0.0));
    requestFeeder = new VoltageOut(0.0);
    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration configIndexer = new TalonFXConfiguration();
    configIndexer.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configIndexer.CurrentLimits.StatorCurrentLimit = 80.0;
    configIndexer.CurrentLimits.StatorCurrentLimitEnable = true;
    configIndexer.CurrentLimits.SupplyCurrentLimit = 40.0;
    configIndexer.CurrentLimits.SupplyCurrentLimitEnable = true;
    configIndexer.Voltage.PeakForwardVoltage = 16.0;
    configIndexer.Voltage.PeakReverseVoltage = 16.0;
    configIndexer.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configIndexer.MotionMagic.MotionMagicExpo_kA = 0.0;
    configIndexer.MotionMagic.MotionMagicExpo_kV = 0.0;
    configIndexer.MotionMagic.MotionMagicAcceleration = 0.0;
    configIndexer.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    configIndexer.Slot0.kP = 0.0;
    configIndexer.Slot0.kI = 0.0;
    configIndexer.Slot0.kD = 0.0;
    configIndexer.Slot0.kS = 0.0;
    configIndexer.Slot0.kV = 0.0;
    configIndexer.Slot0.kA = 0.0;
    indexerMotor.getConfigurator().apply(configIndexer);
  }

  @Override
  public void setIndexerTarget(AngularVelocity velocity) {
    if (!(velocity.in(RPM) == indexerSetPoint.in(RPM))) {
      indexerMotor.setControl(requestIndexer.withVelocity(velocity));
      indexerSetPoint = velocity;
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
    indexerSetPoint = RPM.of(0.0);
    feederSetPoint = Volts.zero();
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    inputs.indexerSupplyCurrent.mut_replace(indexerMotor.getSupplyCurrent().getValue());
    inputs.indexerAngularVelocity.mut_replace(indexerMotor.getVelocity().getValue());
    inputs.indexerSetpoint.mut_replace(indexerSetPoint);

    inputs.feederSupplyCurrent.mut_replace(feederMotor.getSupplyCurrent().getValue());
    inputs.feederVoltage.mut_replace(feederMotor.getMotorVoltage().getValue());
    inputs.feederSetVoltage.mut_replace(feederSetPoint);
  }
}
