package frc.robot.power;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.goals.RobotGoal;
import frc.robot.util.EnumState;
import frc.robot.util.VirtualSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Monitors battery voltage and robot goal to select the active {@link PowerProfile}.
 *
 * <p>Exposes profile triggers via {@link PowerEvents} so SubsystemBehaviors can bind current limit
 * commands reactively.
 */
public class PowerMonitor extends VirtualSubsystem implements PowerEvents {

  private final Supplier<RobotGoal> goalSupplier;
  private final EnumState<PowerProfile> activeProfile =
      new EnumState<>("PowerMonitor/Profile", PowerProfile.NORMAL);

  private boolean inLowVoltage = false;

  public PowerMonitor(Supplier<RobotGoal> goalSupplier) {
    this.goalSupplier = goalSupplier;
  }

  @Override
  public void periodic() {
    double voltage = RobotController.getBatteryVoltage();

    PowerProfile desired = computeDesiredProfile(voltage);
    activeProfile.set(desired);

    Logger.recordOutput("PowerMonitor/BatteryVoltage", voltage);
    Logger.recordOutput("PowerMonitor/InLowVoltage", inLowVoltage);
  }

  private PowerProfile computeDesiredProfile(double voltage) {
    double lowThreshold = PowerBudgetConstants.LOW_VOLTAGE_THRESHOLD.get();
    double hysteresis = PowerBudgetConstants.LOW_VOLTAGE_HYSTERESIS.get();

    if (inLowVoltage) {
      if (voltage > lowThreshold + hysteresis) {
        inLowVoltage = false;
      }
    } else {
      if (voltage < lowThreshold) {
        inLowVoltage = true;
      }
    }

    if (inLowVoltage) {
      return PowerProfile.LOW_VOLTAGE;
    }

    RobotGoal goal = goalSupplier.get();
    if (goal == RobotGoal.SHOOTING || goal == RobotGoal.AIMING) {
      return PowerProfile.SHOOTING;
    }

    return PowerProfile.NORMAL;
  }

  @Override
  public Trigger isNormalProfileTrigger() {
    return activeProfile.is(PowerProfile.NORMAL);
  }

  @Override
  public Trigger isShootingProfileTrigger() {
    return activeProfile.is(PowerProfile.SHOOTING);
  }

  @Override
  public Trigger isLowVoltageProfileTrigger() {
    return activeProfile.is(PowerProfile.LOW_VOLTAGE);
  }
}
