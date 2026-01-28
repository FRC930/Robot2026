package frc.robot.subsystems.climber;

import frc.robot.util.AllEvents;
import frc.robot.util.SubsystemBehavior;

public class ClimberBehavior extends SubsystemBehavior {

  private final ClimberSubsystem climber;

  public ClimberBehavior(ClimberSubsystem climber) {
    this.climber = climber;
  }

  @Override
  public void configure(AllEvents events) {
    events.goals().isIdleTrigger().whileTrue(climber.idle());
    events.goals().isClimbingL0().whileTrue(climber.goToL0Command());
    events.goals().isClimbingL1().whileTrue(climber.goToL1Command());
    events.goals().isClimbingL2().whileTrue(climber.goToL2Command());
    events.goals().isClimbingL3().whileTrue(climber.goToL3Command());
  }
}
