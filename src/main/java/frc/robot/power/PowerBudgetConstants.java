package frc.robot.power;

import frc.robot.util.LoggedTunableNumber;

/** Tunable thresholds for power profile selection. */
public final class PowerBudgetConstants {

  /** Below this voltage, switch to LOW_VOLTAGE profile regardless of goal. */
  public static final LoggedTunableNumber LOW_VOLTAGE_THRESHOLD =
      new LoggedTunableNumber("PowerMonitor/LowVoltageThreshold", 7.5);

  /** Must exceed threshold + hysteresis to exit LOW_VOLTAGE. */
  public static final LoggedTunableNumber LOW_VOLTAGE_HYSTERESIS =
      new LoggedTunableNumber("PowerMonitor/LowVoltageHysteresis", 0.5);

  private PowerBudgetConstants() {}
}
