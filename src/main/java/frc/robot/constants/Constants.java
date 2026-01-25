// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetection.ObjectDetectionType;

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
    NORMAL
  }

  public static final SubsystemMode driveMode = SubsystemMode.NORMAL;
  public static final SubsystemMode shooterMode = SubsystemMode.NORMAL;
  public static final SubsystemMode flywheelMode = SubsystemMode.NORMAL;
  public static final SubsystemMode hoodMode = SubsystemMode.NORMAL;
  public static final SubsystemMode spindexerMode = SubsystemMode.NORMAL;
  public static final SubsystemMode tunnelMode = SubsystemMode.NORMAL;
  public static final SubsystemMode turretMode = SubsystemMode.NORMAL;
  public static final SubsystemMode deployerMode = SubsystemMode.NORMAL;
  public static final SubsystemMode rollerMode = SubsystemMode.NORMAL;
  public static final SubsystemMode climberMode = SubsystemMode.DISABLED;
  public static final SubsystemMode visionGlobalPose = SubsystemMode.NORMAL;
  public static final SubsystemMode visionObjectDetection = SubsystemMode.NORMAL;

  public static final class VisionGlobalPose {}

  public static final class VisionObjectDetection {
    public static final ObjectDetectionType detectionType = ObjectDetectionType.OBJECT;
    public static final Transform3d robotCenterToCamera = new Transform3d(
            -0.2208,
            -0.23495,
            0.98315,
            new Rotation3d(0, Units.degreesToRadians(40), Units.degreesToRadians(180)));
    public static final boolean enableObjectDetectionDebug = false;; // TODO set this
  }
}
