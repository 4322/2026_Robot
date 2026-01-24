// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final boolean buzz = false;

  public static final String logPath = "/home/lvuser/logs";
  public static final long minFreeSpace = 1000000000; // 1 GB
  public static final int dioCoastButton = 0;
  public static final double coastButtonDelaySec = 10.0;

  public static final double brownoutVoltage = 5.75;

  public static final double loopPeriodSecs = 0.02;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Spindexer {
    public static final int spindexerMotorId = 1;
    public static final double supplyCurrentLimit = 0;
    public static final double statorCurrentLimit = 0;
    public static final InvertedValue motorInvert = null;
    public static final NeutralModeValue neutralMode = null;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double indexingVelocityRotationsPerSec = 0;
  }

  public static class Tunnel {}
}
