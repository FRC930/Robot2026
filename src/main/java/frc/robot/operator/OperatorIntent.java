package frc.robot.operator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Interprets driver controller inputs into intent triggers.
 *
 * <p>This class has NO state - it just wraps controller buttons and provides semantic meaning
 * (wantsToScore vs rightTrigger).
 *
 * <p>State lives in RobotGoals. This class just says what the operator is physically doing right
 * now.
 *
 * <p>TODO (students): Map your controller buttons to semantic intents here
 */
public class OperatorIntent implements OperatorIntentEvents {

  private final CommandXboxController driver;

  private static OperatorIntent instance;

  private static int m_port;

  private OperatorIntent(int driverPort) {
    this.driver = new CommandXboxController(driverPort);
  }

  public static OperatorIntent getInstance(int driverPort) {
    if (instance == null) {
      instance = new OperatorIntent(driverPort);
      m_port = driverPort;
    } else {
      // Check someone attempting to get operator controls with different controller
      if (m_port != driverPort) {
        DriverStation.reportWarning(
            "Attempt to get instance with different driver port number " + driverPort,
            Thread.currentThread().getStackTrace());
      }
    }
    return instance;
  }

  @Override
  public Trigger wantsToScoreTrigger() {
    return new Trigger(
        () -> {
          return driver.getHID().getRightBumperButton();
        }); // return driver.rightBumper();
  }

  public CommandXboxController getDriver() {
    return driver;
  }

  // @Override
  // public Trigger wantsToClimbL0() {
  //   return driver.povLeft();
  // }

  // public Trigger wantsToClimbL1() {
  //   return driver.povUp();
  // }

  // @Override
  // public Trigger wantsToClimbL2() {
  //   return driver.povRight();
  // }

  // @Override
  // public Trigger wantsToClimbL3() {
  //   return driver.povDown();
  // }

  @Override
  public Trigger wantsToOuttake() {
    return new Trigger(
        () -> {
          return driver.getHID().getBButton();
        }); // return driver.b();
  }

  // unused, need to get rid of (shooting does the same thing as passing)
  @Override
  public Trigger wantsToPass() {
    return new Trigger(
        () -> {
          return driver.getHID().getBackButton();
        }); // return driver.back();
  }

  @Override
  public Trigger wantsToAim() {
    return new Trigger(
        () -> {
          return driver.getHID().getYButton();
        }); // return driver.y();
  }

  @Override
  public Trigger wantsToIntake() {
    return new Trigger(
        () -> {
          return driver.getHID().getLeftBumperButton();
        }); // return driver.leftBumper();
  }

  // @Override
  // public Trigger wantsToRevIndexer() {
  //   return driver.rightTrigger();
  // }

  @Override
  public Trigger wantsToRaiseIntake() {
    return new Trigger(
        () -> {
          if (driver.getHID().getPOV() == 0) {
            return true;
          } else {
            return false;
          }
        }); // return driver.povUp();
  }

  @Override
  public Trigger wantsToShootNoTolerance() {
    return new Trigger(
        () -> {
          if (driver.getHID().getRightTriggerAxis() >= 0.5) {
            return true;
          } else {
            return false;
          }
        }); // return driver.rightTrigger();
  }
}
