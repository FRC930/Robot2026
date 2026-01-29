package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
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

  private MotionMagicVelocityTorqueCurrentFOC shooterRequest;
  private AngularVelocity shooterSetPoint = RPM.of(0);

  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  public ShooterIOTalonFX(
      int shooterMotorCAN, int followerMotor1CAN, int followerMotor2CAN, CANBus canbus) {
    shooterMotor = new TalonFX(shooterMotorCAN, canbus);
    follower1 = new TalonFX(followerMotor1CAN, canbus);
    follower2 = new TalonFX(followerMotor2CAN, canbus);
    shooterRequest = new MotionMagicVelocityTorqueCurrentFOC(RPM.of(0.0));
    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration configshooter = new TalonFXConfiguration();
    configshooter.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configshooter.CurrentLimits.StatorCurrentLimit = 80.0;
    configshooter.CurrentLimits.StatorCurrentLimitEnable = true;
    configshooter.CurrentLimits.SupplyCurrentLimit = 40.0;
    configshooter.CurrentLimits.SupplyCurrentLimitEnable = true;
    configshooter.Voltage.PeakForwardVoltage = 16.0;
    configshooter.Voltage.PeakReverseVoltage = 16.0;
    configshooter.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configshooter.MotionMagic.MotionMagicExpo_kA = 1.0;
    configshooter.MotionMagic.MotionMagicExpo_kV = 1.0;
    configshooter.MotionMagic.MotionMagicAcceleration = 1.0;
    configshooter.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    configshooter.Slot0.kP = 1.0;
    configshooter.Slot0.kI = 0.0;
    configshooter.Slot0.kD = 0.0;
    configshooter.Slot0.kS = 1.0;
    configshooter.Slot0.kV = 1.0;
    configshooter.Slot0.kA = 1.0;
    shooterMotor.getConfigurator().apply(configshooter);

    TalonFXConfiguration follower1Configuration = new TalonFXConfiguration();
    follower1Configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    follower1Configuration.CurrentLimits.StatorCurrentLimit = 80.0;
    follower1Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    follower1Configuration.CurrentLimits.SupplyCurrentLimit = 40.0;
    follower1Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    follower1Configuration.Voltage.PeakForwardVoltage = 16.0;
    follower1Configuration.Voltage.PeakReverseVoltage = 16.0;
    follower1.getConfigurator().apply(follower1Configuration);
    follower1.setControl(new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Aligned));

    TalonFXConfiguration follower2Configuration = new TalonFXConfiguration();
    follower2Configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    follower2Configuration.CurrentLimits.StatorCurrentLimit = 80.0;
    follower2Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    follower2Configuration.CurrentLimits.SupplyCurrentLimit = 40.0;
    follower2Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    follower2Configuration.Voltage.PeakForwardVoltage = 16.0;
    follower2Configuration.Voltage.PeakReverseVoltage = 16.0;
    follower2.getConfigurator().apply(follower2Configuration);
    follower2.setControl(new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void setShooterTarget(AngularVelocity target) {
    shooterMotor.setControl(shooterRequest.withVelocity(target));
    shooterSetPoint = target;
    // shooterMotor.set(target.in(Volts));
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
  }

  @Override
  public void setGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kG = gains.kG;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    PhoenixUtil.tryUntilOk(5, () -> shooterMotor.getConfigurator().apply(slot0Configs));

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    PhoenixUtil.tryUntilOk(5, () -> shooterMotor.getConfigurator().apply(motionMagicConfigs));
  }
}
