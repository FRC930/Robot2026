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
import frc.robot.aiming.AimingConstants;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;
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

  // Gear tooth counts: main turret gear and two encoder gears (must be co-prime)
  private static final int MAIN_TEETH = 195;
  private static final int ENC1_TEETH = 17;
  private static final int ENC2_TEETH = 15;

  // CRT constants (precomputed)
  // inv(15, 17) = 8  because 15 * 8 = 120 ≡ 1 (mod 17)
  // inv(17, 15) = 8  because 17 * 8 = 136 ≡ 1 (mod 15)
  private static final int CRT_COEFF1 = ENC2_TEETH * 8; // 120
  private static final int CRT_COEFF2 = ENC1_TEETH * 8; // 136
  private static final int CRT_MODULUS = ENC1_TEETH * ENC2_TEETH; // 255

  public TurretIOTalonFX(int motorID, int canCoder1ID, int canCoder2ID, CANBus canbus) {
    motor = new TalonFX(motorID, canbus);
    canCoder1 = new CANcoder(canCoder1ID, canbus);
    canCoder2 = new CANcoder(canCoder2ID, canbus);
    m_setAngle = Degrees.of(0.0);
    request = new MotionMagicTorqueCurrentFOC(0);

    configureTalons();
  }

  /**
   * Computes absolute turret angle from two CANcoder readings using the Chinese Remainder Theorem.
   *
   * <p>Each encoder's gear is co-prime with the other (17 and 15), so CRT uniquely determines the
   * main gear position across all 195 teeth (17 * 15 = 255 > 195).
   *
   * <p>Steps: (1) convert each encoder reading to a tooth index on its gear, (2) CRT to find the
   * unique tooth position on the 195-tooth main gear, (3) add sub-tooth precision from encoder 1.
   *
   * @param e1Deg encoder 1 reading in degrees (0 to 360, on the 17-tooth gear)
   * @param e2Deg encoder 2 reading in degrees (0 to 360, on the 15-tooth gear)
   * @return turret angle in degrees, normalized to [-180, 180]
   */
  public static double calculateTurretAngleFromCANCoderDegrees(double e1Deg, double e2Deg) {
    // Convert encoder degrees to fractional tooth positions on each gear
    double r1 = e1Deg / 360.0 * ENC1_TEETH;
    double r2 = e2Deg / 360.0 * ENC2_TEETH;

    // Round to nearest integer tooth for CRT input
    long i1 = Math.round(r1);
    long i2 = Math.round(r2);

    // Sub-tooth fractional precision (from encoder 1)
    double fraction = r1 - i1;

    // Wrap to valid range [0, n)
    i1 = Math.floorMod(i1, ENC1_TEETH);
    i2 = Math.floorMod(i2, ENC2_TEETH);

    // CRT: unique tooth position on main gear (mod 255)
    int toothPosition = Math.floorMod((int) (i1 * CRT_COEFF1 + i2 * CRT_COEFF2), CRT_MODULUS);

    // Convert to continuous turret angle with sub-tooth precision
    double turretAngle = (toothPosition + fraction) * 360.0 / MAIN_TEETH;

    // Normalize to [-180, 180]
    if (turretAngle > 180.0) {
      turretAngle -= 360.0;
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
    cfg.Feedback.SensorToMechanismRatio = 10.0 / 195.0;
    cfg.Feedback.RotorToSensorRatio = 1.0;
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(new TalonFXConfiguration()));
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(cfg));

    double startAngle =
        calculateTurretAngleFromCANCoderDegrees(
            getCanCoderAngle1().in(Degrees), getCanCoderAngle2().in(Degrees));
    motor.setPosition(startAngle, KCANTIMEOUT);

    // High-frequency signal updates for 250Hz turret thread
    motor.getPosition().setUpdateFrequency(AimingConstants.AIMING_FREQUENCY);
    motor.getVelocity().setUpdateFrequency(AimingConstants.AIMING_FREQUENCY);
    motor.optimizeBusUtilization();
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
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(slot0Configs));

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(motionMagicConfigs));
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
