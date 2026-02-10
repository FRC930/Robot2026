package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.Gains;

public class HoodIOTalonFX implements HoodIO {

  public MotionMagicExpoTorqueCurrentFOC request;

  public TalonFX motor;

  private Angle m_setAngle;

  public HoodIOTalonFX(int motorID, CANBus canbus) {
    motor = new TalonFX(motorID, canbus);
    m_setAngle = Degrees.of(0.0);
    request = new MotionMagicExpoTorqueCurrentFOC(0);
  }

  public void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    cfg.CurrentLimits.SupplyCurrentLimit = 80.0;
    cfg.CurrentLimits.StatorCurrentLimit = 80.0;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    cfg.Feedback.SensorToMechanismRatio =
        46.2; // Combination of a 3:1 Ratio from the Motor Pinion to and a 15.4:1 Ratio Pinion to
    // Hood
    cfg.Feedback.RotorToSensorRatio = 1.0;
  }

  @Override
  public void setHoodTarget(Angle angle) {
    if (angle.in(Degrees) != m_setAngle.in(Degrees)) {
      request = request.withPosition(angle).withSlot(0);
      motor.setControl(request);
      m_setAngle = angle;
    }
  }

  @Override
  public void stop() {}

  @Override
  public void updateInputs(HoodInputs input) {
    input.hoodAngle.mut_replace(motor.getPosition().getValue());
    input.hoodSetAngle.mut_replace(m_setAngle);
  }

  public void setGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kG = gains.kG;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    motor.getConfigurator().apply(slot0Configs);

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    motor.getConfigurator().apply(motionMagicConfigs);
  }
}
