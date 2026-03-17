package frc.robot.power;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Event interface for power profile state, consumed by SubsystemBehaviors via AllEvents. */
public interface PowerEvents {
  Trigger isNormalProfileTrigger();

  Trigger isShootingProfileTrigger();

  Trigger isLowVoltageProfileTrigger();
}
