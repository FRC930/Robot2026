package frc.robot.subsystems.extender;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialFollower;
import com.ctre.phoenix6.controls.DifferentialMotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.DifferentialSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class ExtenderIOTalonFX implements ExtenderIO {
  // Conservative initial Motion Magic constraints (inches/sec, inches/sec^2). Overridden at runtime
  // by ExtenderSubsystem via setMotionConstraints().
  private static final double INITIAL_CRUISE_VELOCITY_IN_PER_SEC = 30.0;
  private static final double INITIAL_ACCELERATION_IN_PER_SEC2 = 50.0;

  public DifferentialMotionMagicVoltage Request;
  public TalonFX extenderMotor;
  public TalonFX followerMotor;

  public ExtenderInputs inputs;

  private Distance m_setPoint = Distance.ofBaseUnits(0, Inches);

  // Last-applied Motion Magic constraints (inches) — avoid redundant config applies.
  private double lastCruiseVelInchPerSec = Double.NaN;
  private double lastAccelInchPerSec2 = Double.NaN;

  private final NeutralOut m_brake = new NeutralOut();

  private static double inchesToRotations(double inches) {
    return inches / ExtenderSubsystem.INCHES_PER_ROT;
  }

  public ExtenderIOTalonFX(int extenderMotorID, int followerMotorID, CANBus canbus) {
    extenderMotor = new TalonFX(extenderMotorID, canbus);
    followerMotor = new TalonFX(followerMotorID, canbus);
    m_setPoint = Inches.of(0.0);
    Request = new DifferentialMotionMagicVoltage(0, 0).withEnableFOC(true);

    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration cfgExtender = new TalonFXConfiguration();
    cfgExtender.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfgExtender.Voltage.PeakForwardVoltage = 4;
    cfgExtender.Voltage.PeakReverseVoltage = -4;
    cfgExtender.CurrentLimits.StatorCurrentLimit = 40;
    cfgExtender.CurrentLimits.StatorCurrentLimitEnable = true;
    cfgExtender.CurrentLimits.SupplyCurrentLimit = 20;
    cfgExtender.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfgExtender.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    cfgExtender.Feedback.SensorToMechanismRatio = ExtenderSubsystem.REDUCTION;

    // Differential sensor config — use follower's built-in encoder for sync
    cfgExtender.DifferentialSensors.DifferentialSensorSource =
        DifferentialSensorSourceValue.RemoteTalonFX_HalfDiff;
    cfgExtender.DifferentialSensors.DifferentialTalonFXSensorID = followerMotor.getDeviceID();

    // Slot1 for difference axis PID (sync control) — tuned on robot
    cfgExtender.Slot1.kP = 0;
    cfgExtender.Slot1.kI = 0;
    cfgExtender.Slot1.kD = 0;

    // Initial Motion Magic profile constraints. Runtime overrides come through
    // setMotionConstraints().
    cfgExtender.MotionMagic.MotionMagicCruiseVelocity =
        inchesToRotations(INITIAL_CRUISE_VELOCITY_IN_PER_SEC);
    cfgExtender.MotionMagic.MotionMagicAcceleration =
        inchesToRotations(INITIAL_ACCELERATION_IN_PER_SEC2);
    lastCruiseVelInchPerSec = INITIAL_CRUISE_VELOCITY_IN_PER_SEC;
    lastAccelInchPerSec2 = INITIAL_ACCELERATION_IN_PER_SEC2;

    PhoenixUtil.tryUntilOk(5, () -> extenderMotor.getConfigurator().apply(cfgExtender));

    TalonFXConfiguration cfgFollower = new TalonFXConfiguration();
    cfgFollower.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfgFollower.Voltage.PeakForwardVoltage = 4;
    cfgFollower.Voltage.PeakReverseVoltage = -4;
    cfgFollower.CurrentLimits.StatorCurrentLimit = 40;
    cfgFollower.CurrentLimits.StatorCurrentLimitEnable = true;
    cfgFollower.CurrentLimits.SupplyCurrentLimit = 20;
    cfgFollower.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfgFollower.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    cfgFollower.Feedback.SensorToMechanismRatio = ExtenderSubsystem.REDUCTION;

    PhoenixUtil.tryUntilOk(5, () -> followerMotor.getConfigurator().apply(cfgFollower));

    // followerMotor.setPosition(0.0);
    followerMotor.setControl(
        new DifferentialFollower(extenderMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    // extenderMotor.setPosition(0.0);
  }

  @Override
  public void updateInputs(ExtenderInputs inputs) {
    double rotations = extenderMotor.getPosition().getValue().in(Rotations);
    inputs.distance.mut_replace(Inches.of(rotations * ExtenderSubsystem.INCHES_PER_ROT));

    double followerRotations = followerMotor.getPosition().getValue().in(Rotations);
    inputs.followerDistance.mut_replace(
        Inches.of(followerRotations * ExtenderSubsystem.INCHES_PER_ROT));

    double diffError = inputs.distance.in(Inches) - inputs.followerDistance.in(Inches);
    inputs.differentialPositionError.mut_replace(Inches.of(diffError));

    inputs.velocity.mut_replace(
        InchesPerSecond.of(
            extenderMotor.getVelocity().getValue().in(RotationsPerSecond)
                * ExtenderSubsystem.INCHES_PER_ROT));
    inputs.setPoint.mut_replace(m_setPoint);
    inputs.supplyCurrent.mut_replace(extenderMotor.getSupplyCurrent().getValue());
    inputs.voltage.mut_replace(extenderMotor.getMotorVoltage().getValue());
  }

  @Override
  public void setExtenderHeight(Distance target) {
    Request =
        Request.withAveragePosition(inchesToRotations(target.in(Inches)))
            .withDifferentialPosition(0)
            .withAverageSlot(0)
            .withDifferentialSlot(1);
    extenderMotor.setControl(Request);
    m_setPoint = target;
  }

  @Override
  public void stop() {
    extenderMotor.setControl(m_brake);
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
    PhoenixUtil.tryUntilOk(5, () -> extenderMotor.getConfigurator().apply(slot0Configs));
  }

  @Override
  public void setDifferentialGains(Gains gains) {
    Slot1Configs slot1Configs = new Slot1Configs();
    slot1Configs.kP = gains.kP;
    slot1Configs.kI = gains.kI;
    slot1Configs.kD = gains.kD;
    slot1Configs.kS = gains.kS;
    slot1Configs.kV = gains.kV;
    slot1Configs.kA = gains.kA;
    PhoenixUtil.tryUntilOk(5, () -> extenderMotor.getConfigurator().apply(slot1Configs));
  }

  @Override
  public void setMotionConstraints(
      double cruiseVelocityInchesPerSec, double accelerationInchesPerSec2) {
    if (cruiseVelocityInchesPerSec == lastCruiseVelInchPerSec
        && accelerationInchesPerSec2 == lastAccelInchPerSec2) {
      return;
    }
    MotionMagicConfigs cfg = new MotionMagicConfigs();
    cfg.MotionMagicCruiseVelocity = inchesToRotations(cruiseVelocityInchesPerSec);
    cfg.MotionMagicAcceleration = inchesToRotations(accelerationInchesPerSec2);
    PhoenixUtil.tryUntilOk(5, () -> extenderMotor.getConfigurator().apply(cfg));
    lastCruiseVelInchPerSec = cruiseVelocityInchesPerSec;
    lastAccelInchPerSec2 = accelerationInchesPerSec2;
  }
}
