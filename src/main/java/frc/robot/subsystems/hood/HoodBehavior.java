package frc.robot.subsystems.hood;

import frc.robot.util.AllEvents;
import frc.robot.util.SubsystemBehavior;

public class HoodBehavior extends SubsystemBehavior {
  private final HoodSubsystem hood;

  public HoodBehavior(HoodSubsystem hood) {
    this.hood = hood;
  }

  @Override
  public void configure(AllEvents events) {
    events.goals().isAimingTrigger().whileTrue(hood.aimCommand());
    events.goals().isPassingTrigger().whileTrue(hood.passCommand());
    events.goals().isIdleTrigger().whileTrue(hood.idleCommand());
  }
}
