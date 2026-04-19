package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {
  TalonFX followIntakeMotor;
  TalonFX leaderIntakeMotor;

  private VelocityVoltage intakeRequest;
  private AngularVelocity intakeSetPoint = RPM.of(0);
  boolean firstTime = true;
  public static AngularVelocity KRACKEN_X60_FOC_MAX_RPM = RPM.of(5784);
  public static double GEAR_RATIO_ROLLERS = 3.0;

  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  public IntakeIOTalonFX(int IntakeLeadMotorCAN, int IntakeFollowMotorCAN, CANBus canbus) {
    leaderIntakeMotor = new TalonFX(IntakeLeadMotorCAN, canbus);
    followIntakeMotor = new TalonFX(IntakeFollowMotorCAN, canbus);
    intakeRequest = new VelocityVoltage(RPM.of(0.0)).withEnableFOC(true).withSlot(0);
    configureTalons();
  }

  public void configureTalons() {
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leaderConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leaderConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderConfig.Voltage.PeakForwardVoltage = 12.0;
    leaderConfig.Voltage.PeakReverseVoltage = -12.0;
    leaderConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO_ROLLERS;
    leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(
        5, () -> leaderIntakeMotor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> leaderIntakeMotor.getConfigurator().apply(leaderConfig));

    TalonFXConfiguration followConfig = new TalonFXConfiguration();
    followConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    followConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    followConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    followConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    followConfig.Voltage.PeakForwardVoltage = 12.0;
    followConfig.Voltage.PeakReverseVoltage = -12.0;
    followConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO_ROLLERS;
    followConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    followConfig.Feedback.RotorToSensorRatio = 1.0;
    PhoenixUtil.tryUntilOk(
        5, () -> followIntakeMotor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> followIntakeMotor.getConfigurator().apply(followConfig));

    followIntakeMotor.setControl(
        new Follower(leaderIntakeMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.rollerVelocity.mut_replace(leaderIntakeMotor.getVelocity().getValue());
    inputs.rollerVelocitySetPoint.mut_replace(intakeSetPoint);
    inputs.rollerSupplyCurrent.mut_replace(leaderIntakeMotor.getSupplyCurrent().getValue());
    inputs.rollerTorqueCurrent.mut_replace(leaderIntakeMotor.getTorqueCurrent().getValue());
    inputs.rollerVoltage.mut_replace(leaderIntakeMotor.getMotorVoltage().getValue());
    inputs.leaderRollerTemp.mut_replace(leaderIntakeMotor.getDeviceTemp().getValue());
    inputs.followerRollerTemp.mut_replace(followIntakeMotor.getDeviceTemp().getValue());
  }

  @Override
  public void stop() {
    leaderIntakeMotor.setControl(m_brake);
    intakeSetPoint = RPM.of(0.0);
    // Doing nothing with extender motor
    // intakeExtenderMotor.setControl(intakeExtenderRequest.withOutput(-5.0));
  }

  @Override
  public void setRollerTargetSpeed(AngularVelocity target) {
    if (target.in(RPM) != intakeSetPoint.in(RPM)) {
      leaderIntakeMotor.setControl(intakeRequest.withVelocity(target));
      intakeSetPoint = target;
      // IntakeMotor.set(target.in(Volts))
    }
  }

  @Override
  public void setRollerGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    PhoenixUtil.tryUntilOk(5, () -> leaderIntakeMotor.getConfigurator().apply(slot0Configs));

    // MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    // motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    // motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    // motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    // motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    // motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    // PhoenixUtil.tryUntilOk(5, () ->
    // leaderIntakeMotor.getConfigurator().apply(motionMagicConfigs));
  }
}
