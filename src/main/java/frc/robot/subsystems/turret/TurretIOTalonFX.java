package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.Gains;
import org.littletonrobotics.junction.Logger;

public class TurretIOTalonFX implements TurretIO {

  public MotionMagicTorqueCurrentFOC request;

  public TalonFX motor;

  private Angle m_setAngle;

  // TODO initialize offset while disabled
  //      while disabled, get initial offset from the 2 external cancoders
  //      Use this offset when doing internal encoder position
  private Angle offset;

  private double encoder1ratio;
  private double encoder2ratio;

  private CANcoder canCoder1;

  private CANcoder canCoder2;

  private final double KCANTIMEOUT = 0.010;
  private static final double GEAR_0_TOOTH_COUNT = 70.0;
  private static final double GEAR_1_TOOTH_COUNT = 36.0;
  private static final double GEAR_2_TOOTH_COUNT = 34.0;

  private static final double SLOPE =
      (GEAR_2_TOOTH_COUNT * GEAR_1_TOOTH_COUNT)
          / ((GEAR_1_TOOTH_COUNT - GEAR_2_TOOTH_COUNT) * GEAR_0_TOOTH_COUNT);

  public TurretIOTalonFX(int motorID, int canCoder1ID, int canCoder2ID, CANBus canbus) {
    motor = new TalonFX(motorID, canbus);
    canCoder1 = new CANcoder(canCoder1ID, canbus);
    canCoder2 = new CANcoder(canCoder2ID, canbus);
    m_setAngle = Degrees.of(0.0);
    request = new MotionMagicTorqueCurrentFOC(0);

    configureTalons();
  }

  public static double calculateTurretAngleFromCANCoderDegrees(double e1, double e2) {
    double difference = e2 - e1;
    if (difference > 250) {
      difference -= 360;
    }
    if (difference < -250) {
      difference += 360;
    }
    difference *= SLOPE;

    double e1Rotations = (difference * GEAR_0_TOOTH_COUNT / GEAR_1_TOOTH_COUNT) / 360.0;
    double e1RotationsFloored = Math.floor(e1Rotations);
    double turretAngle =
        (e1RotationsFloored * 360.0 + e1) * (GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT);
    if (turretAngle - difference < -100) {
      turretAngle += GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT * 360.0;
    } else if (turretAngle - difference > 100) {
      turretAngle -= GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT * 360.0;
    }
    return turretAngle;
  }

  public void configureTalons() {

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.CurrentLimits.SupplyCurrentLimit = 80.0;
    cfg.CurrentLimits.StatorCurrentLimit = 80.0;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // TODO find actual gear ratios & set encoder ratios (math)
    cfg.Feedback.SensorToMechanismRatio = 1.0;
    cfg.Feedback.RotorToSensorRatio = 1.0;

    double startAngle =
        calculateTurretAngleFromCANCoderDegrees(
            getCanCoderAngle1().in(Degrees), getCanCoderAngle2().in(Degrees));
    motor.setPosition(startAngle, KCANTIMEOUT);
  }

  @Override
  public void setTarget(double angle) {
    if (angle != m_setAngle.in(Degrees)) {
      request = request.withPosition(Degrees.of(angle)).withSlot(0);
      motor.setControl(request);
      m_setAngle = Degrees.of(angle);
    }
  }

  @Override
  public void stop() {}

  @Override
  public void updateInputs(TurretInputs inputs) {
    double canCoderAngle =
        calculateTurretAngleFromCANCoderDegrees(
            getCanCoderAngle1().in(Degrees), getCanCoderAngle2().in(Degrees));
    Logger.recordOutput("Turret/AlgorithmOutput", canCoderAngle);

    inputs.turretAngle.mut_replace(motor.getPosition().getValue());
    inputs.turretAngularVelocity.mut_replace(motor.getVelocity().getValue());
    inputs.turretSetAngle.mut_replace(m_setAngle);
    inputs.canCoderAngle1.mut_replace(canCoder1.getAbsolutePosition().getValue());
    inputs.canCoderAngle2.mut_replace(canCoder2.getAbsolutePosition().getValue());
    inputs.turretVoltage.mut_replace(motor.getMotorVoltage().getValue());
    inputs.turretSupplyCurrent.mut_replace(motor.getSupplyCurrent().getValue());
    inputs.turretTorqueCurrent.mut_replace(motor.getTorqueCurrent().getValue());
  }

  public void setGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.GravityType =
        GravityTypeValue.Elevator_Static; // NOTE: horizonatal so ARM not needed
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

  @Override
  public Angle getCanCoderAngle1() {
    return canCoder1.getAbsolutePosition().getValue();
  }

  @Override
  public Angle getCanCoderAngle2() {
    return canCoder2.getAbsolutePosition().getValue();
  }
}
