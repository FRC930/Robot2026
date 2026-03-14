package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface TurretEvents {
  // These triggers take from turret state
  public Trigger isIdleTrigger();

  public Trigger isInToleranceTrigger();
}
