package frc.robot.subsystems.compactor;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public enum CompactorState {
  TESTING(() -> 0.0),
  IDLE(() -> 0.0),
  TOP(new LoggedTunableNumber("Compactor/setCompactorTopPoint", 10)),
  BOTTOM(new LoggedTunableNumber("Compactor/setCompactorBottomPoint", 0));

  private DoubleSupplier m_compactorHeight;

  private CompactorState(DoubleSupplier compactorHeight) {
    m_compactorHeight = compactorHeight;
  }

  public Distance compactorHeight() {
    return Inches.of(m_compactorHeight.getAsDouble());
  }
}
