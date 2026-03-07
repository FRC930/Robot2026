package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class IndexerIOTalonFX implements IndexerIO {

  private VelocityVoltage indexerRequest;
  private VelocityVoltage feederRequest;
  private TalonFX indexerMotor;
  private TalonFX feederMotor;

  private AngularVelocity indexerSetPoint = RPM.of(0);
  private AngularVelocity feederSetPoint = RPM.of(0);

  private static final double SENSOR_MECH_INDEXER = 24;

  private final NeutralOut m_neutralOut = new NeutralOut();

  public IndexerIOTalonFX(int indexerMotorCAN, int feederMotorCAN, CANBus canbus) {
    indexerMotor = new TalonFX(indexerMotorCAN, canbus);
    feederMotor = new TalonFX(feederMotorCAN, canbus);
    indexerRequest = new VelocityVoltage(RPM.of(0.0)).withEnableFOC(false).withSlot(0);
    feederRequest = new VelocityVoltage(RPM.of(0.0)).withEnableFOC(false).withSlot(0);
    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration configIndexer = new TalonFXConfiguration();
    configIndexer.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configIndexer.CurrentLimits.StatorCurrentLimit = 200.0;
    configIndexer.CurrentLimits.StatorCurrentLimitEnable = true;
    configIndexer.CurrentLimits.SupplyCurrentLimit = 22.0;
    configIndexer.CurrentLimits.SupplyCurrentLimitEnable = true;
    configIndexer.Voltage.PeakForwardVoltage = 12.0;
    configIndexer.Voltage.PeakReverseVoltage = -12.0;
    configIndexer.Feedback.SensorToMechanismRatio = SENSOR_MECH_INDEXER;
    configIndexer.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(
        5, () -> indexerMotor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(configIndexer));

    TalonFXConfiguration configFeeder = new TalonFXConfiguration();
    configFeeder.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configFeeder.CurrentLimits.StatorCurrentLimit = 80.0;
    configFeeder.CurrentLimits.StatorCurrentLimitEnable = true;
    configFeeder.CurrentLimits.SupplyCurrentLimit = 40.0;
    configFeeder.CurrentLimits.SupplyCurrentLimitEnable = true;
    configFeeder.Voltage.PeakForwardVoltage = 12.0;
    configFeeder.Voltage.PeakReverseVoltage = -12.0;
    configFeeder.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    PhoenixUtil.tryUntilOk(
        5, () -> feederMotor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(configFeeder));
  }

  @Override
  public void setIndexerTarget(AngularVelocity velocity) {
    if (velocity.in(RPM) != indexerSetPoint.in(RPM)) {
      indexerMotor.setControl(indexerRequest.withVelocity(velocity));
      indexerSetPoint = velocity;
    }
  }

  @Override
  public void setFeederTarget(AngularVelocity velocity) {
    if (velocity.in(RPM) != feederSetPoint.in(RPM)) {
      feederMotor.setControl(feederRequest.withVelocity(velocity));
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
    inputs.indexerTorqueCurrent.mut_replace(indexerMotor.getTorqueCurrent().getValue());

    inputs.feederVelocity.mut_replace(feederMotor.getVelocity().getValue());
    inputs.feederSupplyCurrent.mut_replace(feederMotor.getSupplyCurrent().getValue());
    inputs.feederSetPoint.mut_replace(feederSetPoint);
    inputs.feederVoltage.mut_replace(feederMotor.getMotorVoltage().getValue());
    inputs.feederTorqueCurrent.mut_replace(feederMotor.getTorqueCurrent().getValue());
  }

  public void setIndexerGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    PhoenixUtil.tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(slot0Configs));

    // MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    // motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    // motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    // motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    // motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    // motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    // PhoenixUtil.tryUntilOk(5, () ->
    //   indexerMotor.getConfigurator().apply(motionMagicConfigs));
  }

  public void setFeederGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(slot0Configs));

    // MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    // motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    // motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    // motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    // motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    // motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    // PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(motionMagicConfigs));
  }
}
