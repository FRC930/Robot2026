package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
  TalonFX shooterMotor;
  TalonFX follower1;
  TalonFX follower2;
  TalonFX follower3;

  private VelocityVoltage shooterRequest;
  private AngularVelocity shooterSetPoint = RPM.of(0);

  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();
  private static final double GEAR_RATIO = 1.333; // 24/18

  public ShooterIOTalonFX(
      int shooterMotorCAN,
      int followerMotor1CAN,
      int followerMotor2CAN,
      int followerMotor3CAN,
      CANBus canbus) {
    shooterMotor = new TalonFX(shooterMotorCAN, canbus);
    follower1 = new TalonFX(followerMotor1CAN, canbus);
    follower2 = new TalonFX(followerMotor2CAN, canbus);
    follower3 = new TalonFX(followerMotor3CAN, canbus);
    shooterRequest =
        new VelocityVoltage(RPM.of(0.0)).withEnableFOC(true).withSlot(0).withEnableFOC(true);
    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration configshooter = new TalonFXConfiguration();
    configshooter.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configshooter.CurrentLimits.StatorCurrentLimit = 80.0;
    configshooter.CurrentLimits.StatorCurrentLimitEnable = true;
    configshooter.CurrentLimits.SupplyCurrentLimit = 40.0;
    configshooter.CurrentLimits.SupplyCurrentLimitEnable = true;
    configshooter.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    configshooter.Feedback.RotorToSensorRatio = 1.0;
    configshooter.Voltage.PeakForwardVoltage = 12.0;
    configshooter.Voltage.PeakReverseVoltage = 0.0;
    configshooter.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // configshooter.MotionMagic.MotionMagicExpo_kA = 1.0;
    // configshooter.MotionMagic.MotionMagicExpo_kV = 1.0;
    // configshooter.MotionMagic.MotionMagicAcceleration = 1.0;
    // configshooter.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    PhoenixUtil.tryUntilOk(
        5, () -> shooterMotor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> shooterMotor.getConfigurator().apply(configshooter));

    TalonFXConfiguration follower1Configuration = new TalonFXConfiguration();
    follower1Configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    follower1Configuration.CurrentLimits.StatorCurrentLimit = 80.0;
    follower1Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    follower1Configuration.CurrentLimits.SupplyCurrentLimit = 40.0;
    follower1Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    configshooter.Feedback.RotorToSensorRatio = 1.0;
    follower1Configuration.Voltage.PeakForwardVoltage = 12.0;
    follower1Configuration.Voltage.PeakReverseVoltage = 0.0;
    PhoenixUtil.tryUntilOk(5, () -> follower1.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> follower1.getConfigurator().apply(follower1Configuration));
    follower1.setControl(new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Aligned));

    TalonFXConfiguration follower2Configuration = new TalonFXConfiguration();
    follower2Configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    follower2Configuration.CurrentLimits.StatorCurrentLimit = 80.0;
    follower2Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    follower2Configuration.CurrentLimits.SupplyCurrentLimit = 40.0;
    follower2Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    configshooter.Feedback.RotorToSensorRatio = 1.0;
    follower2Configuration.Voltage.PeakForwardVoltage = 12.0;
    follower2Configuration.Voltage.PeakReverseVoltage = 0.0;
    PhoenixUtil.tryUntilOk(5, () -> follower2.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> follower2.getConfigurator().apply(follower2Configuration));
    follower2.setControl(new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    TalonFXConfiguration follower3Configuration = new TalonFXConfiguration();
    follower3Configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    follower3Configuration.CurrentLimits.StatorCurrentLimit = 80.0;
    follower3Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    follower3Configuration.CurrentLimits.SupplyCurrentLimit = 40.0;
    follower3Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    configshooter.Feedback.RotorToSensorRatio = 1.0;
    follower3Configuration.Voltage.PeakForwardVoltage = 12.0;
    follower3Configuration.Voltage.PeakReverseVoltage = 0.0;
    PhoenixUtil.tryUntilOk(5, () -> follower3.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> follower3.getConfigurator().apply(follower3Configuration));
    follower3.setControl(new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void setShooterTarget(AngularVelocity target) {
    if (target.in(RPM) != shooterSetPoint.in(RPM)) {
      shooterMotor.setControl(shooterRequest.withVelocity(target));
      shooterSetPoint = target;
      // shooterMotor.set(target.in(Volts));
    }
  }

  @Override
  public void stop() {
    shooterMotor.setControl(new NeutralOut());
    shooterSetPoint = RPM.of(0.0);
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    inputs.shooterAngularVelocity.mut_replace(shooterMotor.getVelocity().getValue());
    inputs.shooterSetpoint.mut_replace(shooterSetPoint);
    inputs.shooterTorqueCurrent.mut_replace(shooterMotor.getTorqueCurrent().getValue());
    inputs.shooterVoltage.mut_replace(shooterMotor.getMotorVoltage().getValue());
    inputs.shooterSupplyCurrent.mut_replace(shooterMotor.getSupplyCurrent().getValue());
  }

  @Override
  public void setGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kG = gains.kG;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    PhoenixUtil.tryUntilOk(5, () -> shooterMotor.getConfigurator().apply(slot0Configs));

    // MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    // motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    // motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    // motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    // motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    // motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    // PhoenixUtil.tryUntilOk(5, () -> shooterMotor.getConfigurator().apply(motionMagicConfigs));
  }
}
