package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class IndexerIOTalonFX implements IndexerIO {
  private TalonFX indexerMotor;
  private TalonFX feederMotor;

  private AngularVelocity indexerSetPoint = RPM.of(0);
  private AngularVelocity feederSetPoint = RPM.of(0);

  private final NeutralOut m_neutralOut = new NeutralOut();

  public IndexerIOTalonFX(int indexerMotorCAN, int feederMotorCAN, CANBus canbus) {
    indexerMotor = new TalonFX(indexerMotorCAN, canbus);
    feederMotor = new TalonFX(feederMotorCAN, canbus);
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

     TalonFXConfiguration configFeeder = new TalonFXConfiguration();
    configFeeder.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configFeeder.CurrentLimits.StatorCurrentLimit = 80.0;
    configFeeder.CurrentLimits.StatorCurrentLimitEnable = true;
    configFeeder.CurrentLimits.SupplyCurrentLimit = 40.0;
    configFeeder.CurrentLimits.SupplyCurrentLimitEnable = true;
    configFeeder.Voltage.PeakForwardVoltage = 16.0;
    configFeeder.Voltage.PeakReverseVoltage = 16.0;
    configFeeder.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configFeeder.MotionMagic.MotionMagicExpo_kA = 0.0;
    configFeeder.MotionMagic.MotionMagicExpo_kV = 0.0;
    configFeeder.MotionMagic.MotionMagicAcceleration = 0.0;
    configFeeder.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    configFeeder.Slot0.kP = 0.0;
    configFeeder.Slot0.kI = 0.0;
    configFeeder.Slot0.kD = 0.0;
    configFeeder.Slot0.kS = 0.0;
    configFeeder.Slot0.kV = 0.0;
    configFeeder.Slot0.kA = 0.0;
    feederMotor.getConfigurator().apply(configFeeder);
  }

  @Override
  public void setIndexerTarget(AngularVelocity velocity) {
    if (!(velocity.in(RPM) == indexerSetPoint.in(RPM))) {
      indexerMotor.setControl(new VelocityVoltage(velocity));
      indexerSetPoint = velocity;
    }
  }

  @Override
  public void setFeederTarget(AngularVelocity velocity) {
    if (!(velocity.in(RPM) == indexerSetPoint.in(RPM))) {
      feederMotor.setControl(new VelocityVoltage(indexerSetPoint));
      feederSetPoint = velocity;
    }
  }

  @Override
  public void stop() {
    indexerMotor.setControl(m_neutralOut);
    feederMotor.setControl(m_neutralOut);
    indexerSetPoint = RPM.of(0.0);
    feederSetPoint = RPM.of(0.0);
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    inputs.indexerVelocity.mut_replace(indexerMotor.getVelocity().getValue());
    inputs.indexerSupplyCurrent.mut_replace(indexerMotor.getSupplyCurrent().getValue());
    inputs.indexerSetPoint.mut_replace(indexerSetPoint);
    inputs.indexerVoltage.mut_replace(indexerMotor.getMotorVoltage().getValue());

    inputs.feederVelocity.mut_replace(feederMotor.getVelocity().getValue());
    inputs.feederSupplyCurrent.mut_replace(feederMotor.getSupplyCurrent().getValue());
    inputs.feederSetPoint.mut_replace(feederSetPoint);
    inputs.feederVoltage.mut_replace(feederMotor.getMotorVoltage().getValue());
  
  }
  
  public void setIndexerGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kG = gains.kG;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    PhoenixUtil.tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(slot0Configs));

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    PhoenixUtil.tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(motionMagicConfigs));
  }
public void setFeederGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kG = gains.kG;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(slot0Configs));

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(motionMagicConfigs));
  }
}
