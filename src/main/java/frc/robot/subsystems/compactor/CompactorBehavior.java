package frc.robot.subsystems.compactor;

import frc.robot.util.AllEvents;
import frc.robot.util.SubsystemBehavior;

public class CompactorBehavior extends SubsystemBehavior {
  private final CompactorSubsystem compactor;

  public CompactorBehavior(CompactorSubsystem compactor) {
    this.compactor = compactor;
  }

  @Override
  public void configure(AllEvents events) {
    events
        .goals()
        .isShootingTrigger()
        .whileTrue(compactor.goToBottomCommand())
        .onFalse(compactor.idleCommand());
    events
        .goals()
        .isIntakingTrigger()
        .whileTrue(compactor.goToTopCommand())
        .onFalse(compactor.idleCommand());
    events
        .goals()
        .isRaisingCompactorTrigger()
        .whileTrue(compactor.raiseCompactorCommand())
        .onFalse(compactor.idleCommand());
    events.goals()
        .isLoweringCompactorTrigger()
        .whileTrue(compactor.lowerComactorCommand())
        .onFalse(compactor.idleCommand());
  }
}
