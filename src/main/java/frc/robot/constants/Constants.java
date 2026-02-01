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

  public enum SubsystemMode {
    DISABLED,
    NORMAL,
    TUNING,
    DRIVETUNING
  }

  public static final SubsystemMode driveMode = SubsystemMode.NORMAL;
  public static final SubsystemMode shooterMode = SubsystemMode.NORMAL;
  public static final SubsystemMode flywheelMode = SubsystemMode.NORMAL;
  public static final SubsystemMode hoodMode = SubsystemMode.NORMAL;
  public static final SubsystemMode spindexerMode = SubsystemMode.NORMAL;
  public static final SubsystemMode tunnelMode = SubsystemMode.NORMAL;
  public static final SubsystemMode turretMode = SubsystemMode.NORMAL;
  public static final SubsystemMode deployerMode = SubsystemMode.NORMAL;
  public static final SubsystemMode rollersMode = SubsystemMode.NORMAL;
  public static final SubsystemMode climberMode = SubsystemMode.DISABLED;
  public static final SubsystemMode visionGlobalPose = SubsystemMode.NORMAL;
  public static final SubsystemMode visionObjectDetection = SubsystemMode.NORMAL;

  public class Deployer {
    // 0 degrees is stowed postion
    // postive degrees when extending
    public static final double retractDeg = 0; // TODO
    public static final double extendDeg = 95; // TODO
    public static final double maxGravityDegrees = 65; // TODO
    public static final int motorId = 1; // TODO
    public static final double statorCurrentLimit = 60;
    public static final double supplyCurrentLimit = 40;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final InvertedValue motorInvert = InvertedValue.Clockwise_Positive;
    public static final double kP = 1; // TODO
    public static final double kG = 2; // TODO
    public static final double kI = 0;
    public static final double kD = 0;
    public static final int canCoderID = 1; // TODO
    public static final double ratioRS = 2; //TODO
    public static final double ratioSM = 1; //TODO
  }

  public class Rollers {
    public static final double voltageIntake = 50; // TODO
    public static final double voltageEject = -50; // TODO
    public static final int motorId = 1; // TODO
    public static final double statorCurrentLimit = 60; // TODO
    public static final double supplyCurrentLimit = 40; // TODO
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final InvertedValue motorInvert = InvertedValue.Clockwise_Positive;
  }
}
