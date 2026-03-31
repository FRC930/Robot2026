package frc.robot.subsystems.compactor;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class CompactorIOTalonFX implements CompactorIO {
  public VelocityVoltage compactorVelocityRequest;
  public MotionMagicTorqueCurrentFOC Request;
  public TalonFX compactorMotor;

  private AngularVelocity compactorSetSpeed = RPM.of(0);

  public CompactorInputs inputs;

  private Distance m_setPoint = Distance.ofBaseUnits(0, Inches);

  private final NeutralOut m_brake = new NeutralOut();

  public CompactorIOTalonFX(int motorID, CANBus canbus) {
    compactorMotor = new TalonFX(motorID, canbus);
    m_setPoint = Inches.of(0.0);
    Request = new MotionMagicTorqueCurrentFOC(0);

    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Voltage.PeakForwardVoltage = 12;
    cfg.Voltage.PeakReverseVoltage = 12;
    cfg.CurrentLimits.StatorCurrentLimit = 80;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 30;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    cfg.Feedback.SensorToMechanismRatio = CompactorSubsystem.REDUCTION;

    PhoenixUtil.tryUntilOk(5, () -> compactorMotor.getConfigurator().apply(cfg));
  }

  @Override
  public void updateInputs(CompactorInputs inputs) {
    double rotations = compactorMotor.getPosition().getValue().in(Rotations);
    inputs.distance.mut_replace(Inches.of(rotations * CompactorSubsystem.INCHES_PER_ROT));
    inputs.velocity.mut_replace(
        InchesPerSecond.of(compactorMotor.getVelocity().getValue().in(RotationsPerSecond)));
    inputs.setPoint.mut_replace(m_setPoint);
    inputs.supplyCurrent.mut_replace(compactorMotor.getSupplyCurrent().getValue());
    inputs.voltage.mut_replace(compactorMotor.getMotorVoltage().getValue());
  }

  @Override
  public void setCompactorHeight(Distance target) {
    Request =
        Request.withPosition(target.in(Inches) / CompactorSubsystem.INCHES_PER_ROT).withSlot(0);
    compactorMotor.setControl(Request);
    m_setPoint = target;
  }

  @Override
  public void setCompactorVelocity(AngularVelocity velocity) {
    if (velocity.in(RPM) != compactorSetSpeed.in(RPM)) {
      compactorMotor.setControl(compactorVelocityRequest.withVelocity(velocity));
      compactorSetSpeed = velocity;
    }
  }

  @Override
  public void stop() {
    compactorMotor.setControl(m_brake);
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
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    PhoenixUtil.tryUntilOk(5, () -> compactorMotor.getConfigurator().apply(slot0Configs));

    // MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    // motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    // motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    // motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    // motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    // motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    // PhoenixUtil.tryUntilOk(5, () -> compactorMotor.getConfigurator().apply(motionMagicConfigs));
  }
}
