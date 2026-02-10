package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  TalonFX followIntakeMotor;
  TalonFX leaderIntakeMotor;

  TalonFX intakeExtenderMotor;

  private VoltageOut intakeRequest;
  private Voltage intakeSetPoint = Volts.of(0);
  private VoltageOut intakeExtenderRequest;
  private Voltage intakeExtenderSetPoint = Volts.of(0);

  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  public IntakeIOTalonFX(
      int IntakeLeadMotorCAN, int IntakeFollowMotorCAN, int IntakeExtenderMotorCAN, CANBus canbus) {
    leaderIntakeMotor = new TalonFX(IntakeLeadMotorCAN, canbus);
    followIntakeMotor = new TalonFX(IntakeFollowMotorCAN, canbus);
    intakeExtenderMotor = new TalonFX(IntakeExtenderMotorCAN, canbus);
    intakeRequest = new VoltageOut(0.0);
    intakeExtenderRequest = new VoltageOut(0.0);
    configureTalons();
  }

  public void configureTalons() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 10.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Voltage.PeakForwardVoltage = 16.0;
    config.Voltage.PeakReverseVoltage = 16.0;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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
    inputs.intakeVoltage.mut_replace(leaderIntakeMotor.getMotorVoltage().getValue());
    inputs.intakeSetVoltage.mut_replace(intakeSetPoint);
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
    intakeExtenderMotor.setControl(intakeExtenderRequest.withOutput(-5.0));
  }

  @Override
  public void setIntakeTarget(Voltage target) {
    if (target.in(Volts) != intakeSetPoint.in(Volts)) {
      leaderIntakeMotor.setControl(intakeRequest.withOutput(target));
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
}
