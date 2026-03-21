package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public enum FeederState {
  TESTING(() -> 0.0),
  IDLE(() -> 0.0),
  FEEDING(new LoggedTunableNumber("Indexer/setFeederPoint", 4000)),
  REVERSING(new LoggedTunableNumber("Indexer/setFeederPointReverse", -2000));

  private DoubleSupplier m_feederVelocity;

  private FeederState(DoubleSupplier feederVelocity) {
    m_feederVelocity = feederVelocity;
  }

  public AngularVelocity feederVelocity() {
    return RPM.of(m_feederVelocity.getAsDouble());
  }
}
