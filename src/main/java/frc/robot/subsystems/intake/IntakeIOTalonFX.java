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
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leaderConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leaderConfig.CurrentLimits.SupplyCurrentLimit = 10.0;
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderConfig.Voltage.PeakForwardVoltage = 16.0;
    leaderConfig.Voltage.PeakReverseVoltage = 16.0;
    leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leaderConfig.MotionMagic.MotionMagicExpo_kA = 1.0;
    leaderConfig.MotionMagic.MotionMagicExpo_kV = 1.0;
    leaderConfig.MotionMagic.MotionMagicAcceleration = 1.0;
    leaderConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    PhoenixUtil.tryUntilOk(
        5, () -> leaderIntakeMotor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> leaderIntakeMotor.getConfigurator().apply(leaderConfig));

    TalonFXConfiguration followConfig = new TalonFXConfiguration();
    followConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    followConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    followConfig.CurrentLimits.SupplyCurrentLimit = 10.0;
    followConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    followConfig.Voltage.PeakForwardVoltage = 16.0;
    followConfig.Voltage.PeakReverseVoltage = 16.0;
    followConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    followConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO; // TODO: Value
    followConfig.Feedback.RotorToSensorRatio = 1.0;
    PhoenixUtil.tryUntilOk(
        5, () -> followIntakeMotor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> followIntakeMotor.getConfigurator().apply(followConfig));

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            followIntakeMotor.setControl(
                new Follower(leaderIntakeMotor.getDeviceID(), MotorAlignmentValue.Opposed)));

    TalonFXConfiguration configIntakeExtender = new TalonFXConfiguration();
    configIntakeExtender.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configIntakeExtender.CurrentLimits.StatorCurrentLimit = 80.0;
    configIntakeExtender.CurrentLimits.StatorCurrentLimitEnable = true;
    configIntakeExtender.CurrentLimits.SupplyCurrentLimit = 10.0;
    configIntakeExtender.CurrentLimits.SupplyCurrentLimitEnable = true;
    configIntakeExtender.Voltage.PeakForwardVoltage = 16.0;
    configIntakeExtender.Voltage.PeakReverseVoltage = 16.0;
    configIntakeExtender.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    PhoenixUtil.tryUntilOk(
        5, () -> intakeExtenderMotor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(
        5, () -> intakeExtenderMotor.getConfigurator().apply(configIntakeExtender));
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.rollerVelocity.mut_replace(leaderIntakeMotor.getVelocity().getValue());
    inputs.rollerVelocitySetPoint.mut_replace(intakeSetPoint);
    inputs.rollerSupplyCurrent.mut_replace(leaderIntakeMotor.getSupplyCurrent().getValue());
    inputs.rollerTorqueCurrent.mut_replace(leaderIntakeMotor.getTorqueCurrent().getValue());
    inputs.rollerVoltage.mut_replace(leaderIntakeMotor.getMotorVoltage().getValue());

    inputs.extenderVoltage.mut_replace(intakeExtenderMotor.getMotorVoltage().getValue());
    inputs.extenderVoltageSetPoint.mut_replace(intakeExtenderSetPoint);
    inputs.extenderSupplyCurrent.mut_replace(intakeExtenderMotor.getSupplyCurrent().getValue());
    inputs.extenderTorqueCurrent.mut_replace(intakeExtenderMotor.getTorqueCurrent().getValue());
    // Used for 3d model in advantage scope TODO MAY WANT PID extenderEmulatedAngle
    inputs.extenderEmulatedAngle.mut_replace(
        IntakeIOSim.emulateVoltsToRadians(intakeExtenderMotor.getMotorVoltage().getValue()));
    inputs.extenderEmulatedSetAngle.mut_replace(
        IntakeIOSim.emulateVoltsToRadians(intakeExtenderSetPoint));
  }

  @Override
  public void stop() {
    leaderIntakeMotor.setControl(m_brake);
    intakeSetPoint = RPM.of(0.0);
    intakeExtenderMotor.setControl(intakeExtenderRequest.withOutput(-5.0));
  }

  @Override
  public void setRollerTargetSpeed(AngularVelocity target) {
    if (target.in(RPM) != intakeSetPoint.in(RPM)) {
      leaderIntakeMotor.setControl(intakeRequest.withVelocity(target).withSlot(0));
      intakeSetPoint = target;
      // IntakeMotor.set(target.in(Volts))
    }
  }

  @Override
  public void setExtenderTargetVolts(Voltage target) {
    if (intakeExtenderSetPoint.in(Volts) != target.in(Volts)) {
      intakeExtenderMotor.setControl(intakeExtenderRequest.withOutput(target));
      intakeExtenderSetPoint = target;
    }
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
