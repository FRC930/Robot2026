// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final boolean overrideEnableLoggedTunableNumbers = false;

  public static final boolean isInElim = DriverStation.getMatchType() == MatchType.Elimination;
  public static final boolean isInQual = DriverStation.getMatchType() == MatchType.Qualification;
  public static final boolean isInPrac = DriverStation.getMatchType() == MatchType.Practice;
  public static final boolean isInMatch = isInElim || isInQual;

  /** Enable tuning mode to allow real-time parameter adjustment from SmartDashboard */
  public static final boolean tuningMode = !isInMatch || overrideEnableLoggedTunableNumbers;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
