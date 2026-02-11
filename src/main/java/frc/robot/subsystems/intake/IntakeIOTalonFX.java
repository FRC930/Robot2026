package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {
  TalonFX followIntakeMotor;
  TalonFX leaderIntakeMotor;

  TalonFX intakeExtenderMotor;

  private VelocityVoltage intakeRequest;
  private AngularVelocity intakeSetPoint = RPM.of(0);
  private VoltageOut intakeExtenderRequest;
  private Voltage intakeExtenderSetPoint = Volts.of(0);
  public static AngularVelocity KRACKEN_X60_FOC_MAX_RPM = RPM.of(5784);
  public static double GEAR_RATIO = 3.0; // TODO: May change

  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  public IntakeIOTalonFX(
      int IntakeLeadMotorCAN, int IntakeFollowMotorCAN, int IntakeExtenderMotorCAN, CANBus canbus) {
    leaderIntakeMotor = new TalonFX(IntakeLeadMotorCAN, canbus);
    followIntakeMotor = new TalonFX(IntakeFollowMotorCAN, canbus);
    intakeExtenderMotor = new TalonFX(IntakeExtenderMotorCAN, canbus);
    intakeRequest = new VelocityVoltage(RPM.of(0.0)).withEnableFOC(true);
    intakeExtenderRequest = new VoltageOut(0.0);
    configureTalons();
  }

  public void configureTalons() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 10.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Voltage.PeakForwardVoltage = 16.0;
    config.Voltage.PeakReverseVoltage = 16.0;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotionMagic.MotionMagicExpo_kA = 1.0;
    config.MotionMagic.MotionMagicExpo_kV = 1.0;
    config.MotionMagic.MotionMagicAcceleration = 1.0;
    config.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    followIntakeMotor.getConfigurator().apply(config);

    config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 10.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Voltage.PeakForwardVoltage = 16.0;
    config.Voltage.PeakReverseVoltage = 16.0;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO; // TODO: Value
    config.Feedback.RotorToSensorRatio = 1.0;
    leaderIntakeMotor.getConfigurator().apply(config);

    followIntakeMotor.setControl(
        new Follower(leaderIntakeMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    TalonFXConfiguration configIntakeExtender = new TalonFXConfiguration();
    configIntakeExtender.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configIntakeExtender.CurrentLimits.StatorCurrentLimit = 80.0;
    configIntakeExtender.CurrentLimits.StatorCurrentLimitEnable = true;
    configIntakeExtender.CurrentLimits.SupplyCurrentLimit = 10.0;
    configIntakeExtender.CurrentLimits.SupplyCurrentLimitEnable = true;
    configIntakeExtender.Voltage.PeakForwardVoltage = 16.0;
    configIntakeExtender.Voltage.PeakReverseVoltage = 16.0;
    configIntakeExtender.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeExtenderMotor.getConfigurator().apply(configIntakeExtender);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.intakeAngularVelocity.mut_replace(leaderIntakeMotor.getVelocity().getValue());
    inputs.intakeSetAngularVelocity.mut_replace(intakeSetPoint);
    inputs.intakeSupplyCurrent.mut_replace(leaderIntakeMotor.getSupplyCurrent().getValue());

    inputs.intakeExtenderVoltage.mut_replace(intakeExtenderMotor.getMotorVoltage().getValue());
    inputs.intakeExtenderSetVoltage.mut_replace(intakeExtenderSetPoint);
    inputs.intakeExtenderSupplyCurrent.mut_replace(
        intakeExtenderMotor.getSupplyCurrent().getValue());
    // inputs.intakeExtenderAngle.(angle(0.0)); // TODO for replay
  }

  @Override
  public void stop() {
    leaderIntakeMotor.setControl(m_brake);
    intakeSetPoint = RPM.of(0.0);
    intakeExtenderMotor.setControl(intakeExtenderRequest.withOutput(-5.0));
  }

  @Override
  public void setIntakeTarget(AngularVelocity target) {
    if (target.in(RPM) != intakeSetPoint.in(RPM)) {
      leaderIntakeMotor.setControl(intakeRequest.withVelocity(target).withSlot(0));
      intakeSetPoint = target;
      // IntakeMotor.set(target.in(Volts))
    }
  }

  @Override
  public void setIntakeExtenderTarget(Voltage target) {
    if (intakeExtenderSetPoint.in(Volts) != target.in(Volts)) {
      intakeExtenderMotor.setControl(intakeExtenderRequest.withOutput(target));
      intakeExtenderSetPoint = target;
    }
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
    PhoenixUtil.tryUntilOk(5, () -> leaderIntakeMotor.getConfigurator().apply(slot0Configs));

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    PhoenixUtil.tryUntilOk(5, () -> leaderIntakeMotor.getConfigurator().apply(motionMagicConfigs));
  }
}
