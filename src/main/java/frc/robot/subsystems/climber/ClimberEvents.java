package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ClimberEvents {

  public Trigger goToL1Trigger();

  public Trigger goToL2Trigger();

  public Trigger goToL3Trigger();

  public Trigger goToIdleTrigger();
}
