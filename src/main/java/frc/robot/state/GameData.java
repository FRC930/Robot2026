package frc.robot.state;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import frc.robot.util.VirtualSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/** Exposes GameData states and trigger to see if activeShooting for behaviors to react to. */
public class GameData extends VirtualSubsystem {

  private static GameData instance;

  public enum GameDataStates {
    INVALID,
    AUTO,
    EXTRA, // Extra time in transation if teleop configured wrong for time left (too much time cfg
    // for teleop)
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME
  }

  private static final String LOGGER_PARENT = "GameData";
  public final EnumState<GameDataStates> m_state =
      new EnumState<>(LOGGER_PARENT + "/States", GameDataStates.INVALID);

  // public GameDataStates m_state = GameDataStates.INVALID;

  public static final double SECONDS_PER_SHIFT = 25.0;
  public static final double SECONDS_PER_TRANSITION = 10.0;
  public static final double SECONDS_PER_ENDGAME = 30.0;

  // Auto             20sec  20.0  20-0  (NOT USED given only use Timer in teleop)
  public static final double SECONDS_PER_AUTO = 20.0;

  // TELEOP TIMELEFT BASE ON STAGE
  // END GAME 30sec   30sec  30.0  0:30-0:00
  public static final double TIMELEFT_ENDGAME = SECONDS_PER_ENDGAME;
  // Shift 4          25sec  55.0  0:55-0:30
  public static final double TIMELEFT_SHIFT4 = TIMELEFT_ENDGAME + SECONDS_PER_SHIFT;
  // Shift 3          25sec  80.0  1:20-0:55
  public static final double TIMELEFT_SHIFT3 = TIMELEFT_SHIFT4 + SECONDS_PER_SHIFT;
  // Shift 2          25sec  105.0 1:45-1:20
  public static final double TIMELEFT_SHIFT2 = TIMELEFT_SHIFT3 + SECONDS_PER_SHIFT;
  // Shift 1          25sec  130.0 2:10-1:45
  public static final double TIMELEFT_SHIFT1 = TIMELEFT_SHIFT2 + SECONDS_PER_SHIFT;
  // Transition shift 10sec  140.0 2:20-2:10
  public static final double TIMELEFT_TRANSITION = TIMELEFT_SHIFT1 + SECONDS_PER_TRANSITION;

  // Conditional state of Alliance which score the most during auto
  private String m_gameMessage = null;
  private Alliance m_allianceScoredTheMost = null;
  private Timer m_timer = null;
  private boolean m_lastActive = true;
  private double m_timeLeftForNextStage = 0.0;
  private int m_counter = 0;
  private double m_teleopTimeLeft = 0.0;

  private GameData() {
    logGameData(m_gameMessage);
  }

  public static GameData getInstance() {
    if (instance == null) {
      instance = new GameData();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // No periodic updates needed - triggers poll DriverStation directly
    if (DriverStation.isDisabled()
        && (m_timer != null || m_state.get() != GameDataStates.INVALID)) {
      resetState();
    } else {
      determineIfActiveShootingAndLog();
    }
  }

  private void resetState() {
    m_timer = null;
    resetGameData();
    m_state.set(GameDataStates.INVALID);
    m_counter = 0;
    m_teleopTimeLeft = -1.0;
  }

  private void resetGameData() {
    if (m_gameMessage != null) {
      m_allianceScoredTheMost = null;
      m_gameMessage = null;
      logGameData(m_gameMessage);
    }
  }

  private void logGameData(String gameData) {
    String gData;
    if (gameData == null || gameData.length() == 0) {
      gData = "EMPTY"; // Either unread or actually empty
    } else {
      gData = gameData;
    }
    Logger.recordOutput(LOGGER_PARENT + "/GameData", gData);
  }

  private boolean determineIfActiveShootingAndLog() {
    boolean active = determineIfActiveShooting();
    if (m_lastActive != active) {
      Logger.recordOutput(LOGGER_PARENT + "/AllowedShooting", active);
      m_lastActive = active;
    }
    return active;
  }

  private boolean determineIfActiveShooting() {
    // Only active if enabled
    if (DriverStation.isEnabled()) {
      if (DriverStation.isAutonomous()) {
        m_state.set(GameDataStates.AUTO);
        // Is always active shooting during auto
        return true;
      } else if (DriverStation.isTeleopEnabled()) {
        m_teleopTimeLeft = getTeleopTimeLeft();
        // NOTE: We get GAMEDATA as soon as possible IN teleop)
        checkGameData();
        if (m_teleopTimeLeft <= 0.0) {
          m_state.set(GameDataStates.INVALID);
          m_timeLeftForNextStage = 0.0;
          // If no time left not active shooting
          return false;
        }
        if (m_teleopTimeLeft < TIMELEFT_ENDGAME) {
          m_state.set(GameDataStates.ENDGAME);
          m_timeLeftForNextStage = 0.0;
          // Is always active shooting during end game
          return true;
        }
        if (m_teleopTimeLeft < TIMELEFT_SHIFT4) {
          m_state.set(GameDataStates.SHIFT4);
          m_timeLeftForNextStage = TIMELEFT_ENDGAME;
          return checkGameDataForShift(false);
        }
        if (m_teleopTimeLeft < TIMELEFT_SHIFT3) {
          m_state.set(GameDataStates.SHIFT3);
          m_timeLeftForNextStage = TIMELEFT_SHIFT4;
          return checkGameDataForShift(true);
        }
        if (m_teleopTimeLeft < TIMELEFT_SHIFT2) {
          m_state.set(GameDataStates.SHIFT2);
          m_timeLeftForNextStage = TIMELEFT_SHIFT3;
          return checkGameDataForShift(false);
        }
        if (m_teleopTimeLeft < TIMELEFT_SHIFT1) {
          m_state.set(GameDataStates.SHIFT1);
          m_timeLeftForNextStage = TIMELEFT_SHIFT2;
          return checkGameDataForShift(true);
        }
        if (m_teleopTimeLeft <= TIMELEFT_TRANSITION) {
          m_state.set(GameDataStates.TRANSITION);
          m_timeLeftForNextStage = TIMELEFT_SHIFT1;
          // Is always active shooting during transition
          return true;
        } else {
          // If Time is greater that what should have for teleop
          m_state.set(GameDataStates.EXTRA);
          m_timeLeftForNextStage = TIMELEFT_TRANSITION;
          return true;
        }
      } else {
        // if not enabled  teleop not active
        resetState();
        // Reset so next time re-enable can get new values
        // NOTE: Using periodic  to reset/This did not reset timer since I believe Command Schedule
        // not running so never runs this
        return false;
      }
    }
    return false;
  }

  private void logGameState(GameDataStates gameState) {
    // So don't update smartdashboard each time
    if (m_state.get() == null || m_state.get() != gameState) {
      Logger.recordOutput(LOGGER_PARENT + "/GameState", gameState);
    }
    m_state.set(gameState);
  }

  private double getTeleopTimeLeft() {
    /* When connected to the real field, this number only changes in full integer increments, and always counts down.
      When the DS is in practice mode, this number is a floating point number, and counts down.
      When the DS is in teleop or autonomous mode, this number is a floating point number, and counts up.
    */
    double timeLeft = DriverStation.getMatchTime();
    if (timeLeft < 0.0) {
      if (DriverStation.isFMSAttached()) {
        // If NO MatchTime Available Simulation Run with Driverstation Pratice match
        return timeLeft;
      } else {
        if (m_timer == null) {
          m_timer = new Timer();
          m_timer.start();
        }
        timeLeft = TIMELEFT_TRANSITION - m_timer.get();
      }
    }

    if ((m_counter++ % 50) == 0) {
      // TODO many be performance issue (logging each time)
      Logger.recordOutput(LOGGER_PARENT + "/TeleopTimeLeft", timeLeft);
      Logger.recordOutput(
          LOGGER_PARENT + "/TeleopTimeLeftInStage", timeLeft - m_timeLeftForNextStage);
    }
    return timeLeft;
  }

  private void checkGameData() {
    // FMS relays the ALLIANCE who scored more FUEL during AUTO, or the ALLIANCE selected by FMS,
    // to all OPERATOR CONSOLES simultaneously at the start of TELEOP
    if (m_gameMessage == null) {
      // NOTE:  we don't get GameData until transition Teleop starts
      // WARNING: FMS MAY NOT EVER GIVEN US GAMEDATA SEE MANUAL see TEAM UPDATES
      // 10.2.c "It is not an ARENA FAULT if FMS Game Data is not sent, not received, or if
      // delayed. Incorrect Game Data being sent would be considered an ARENA FAULT."
      // TODO Create  autochoice and put on smartDashboard  to have codriver provide gamedata if
      // NEVER a getGameSpecificMessage() in SHIFT1
      String gameMessage = DriverStation.getGameSpecificMessage();
      boolean msgHasLength = (gameMessage.length() > 0);
      if (msgHasLength && gameMessage.charAt(0) == 'B') {
        m_allianceScoredTheMost = Alliance.Blue;
      } else {
        if (msgHasLength && gameMessage.charAt(0) == 'R') {
          m_allianceScoredTheMost = Alliance.Red;
        } else {
          // NOTE: Could be repeatedly log if empty or invalid
          logGameData(gameMessage);
          return;
        }
      }
      m_gameMessage = gameMessage;
      logGameData(m_gameMessage);
    }
  }

  private boolean checkGameDataForShift(boolean shouldBeActiveIfScoredMost) {
    // NOTE: could be perforance hit (could cache alliance)
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    if (allianceOptional.isPresent()) {
      // FMS relays the ALLIANCE who scored more FUEL during AUTO, or the ALLIANCE selected by FMS,
      // to all OPERATOR CONSOLES simultaneously at the start of TELEOP
      if (m_gameMessage != null) {
        if (m_allianceScoredTheMost == allianceOptional.get()) {
          return shouldBeActiveIfScoredMost;
        } else {
          // If did not score the most do opposite
          return !shouldBeActiveIfScoredMost;
        }
      } else {
        checkGameData();
      }
    }
    return true;
  }

  public Trigger isActiveShootingTrigger() {
    return new Trigger(() -> m_lastActive);
  }

  public Trigger isCountDownToNextStage(double seconds) {
    return new Trigger(
        () -> {
          return m_teleopTimeLeft > 0.0 && (m_teleopTimeLeft - m_timeLeftForNextStage) <= seconds;
        });
  }

  // TODO  need trigger for each state?  since has EnumState triggers() allows to do the following
  // gameData.m_state.is(GameData.GameDataStates.SHIFT1)

}
