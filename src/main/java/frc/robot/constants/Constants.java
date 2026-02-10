package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
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
    DRIVE_TUNING
  }

  public static final SubsystemMode driveMode = SubsystemMode.NORMAL;
  public static final SubsystemMode flywheelMode = SubsystemMode.NORMAL;
  public static final SubsystemMode hoodMode = SubsystemMode.NORMAL;
  public static final SubsystemMode spindexerMode = SubsystemMode.NORMAL;
  public static final SubsystemMode tunnelMode = SubsystemMode.NORMAL;
  public static final SubsystemMode turretMode = SubsystemMode.NORMAL;
  public static final SubsystemMode deployerMode = SubsystemMode.NORMAL;
  public static final SubsystemMode rollerMode = SubsystemMode.NORMAL;
  public static final SubsystemMode intakeMode = SubsystemMode.NORMAL;
  public static final SubsystemMode climberMode = SubsystemMode.DISABLED;
  public static final SubsystemMode ledMode = SubsystemMode.NORMAL;
  public static final SubsystemMode visionGlobalPose = SubsystemMode.NORMAL;
  public static final SubsystemMode visionObjectDetection = SubsystemMode.NORMAL;

  public static class Spindexer {
    public static final boolean dynamicVelocity = true;
    public static final double dynamicVelocityPercent = 0.9; // TODO tune

    public static final int spindexerMotorId = 1;
    public static final double supplyCurrentLimit = 40; // TODO
    public static final double statorCurrentLimit = 60;
    public static final InvertedValue motorInvert =
        InvertedValue.Clockwise_Positive; // TODO set these
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double indexingMechanismRotationsPerSec = 3; // TODO
    public static final double stoppedMechanismRotationsPerSec = 0.1; // TODO

    public static final double motorToMechanismRatio = 12.0;
  }

  public static class Tunnel {
    public static final boolean dynamicVelocity = true;
    public static final double dynamicVelocityPercent = 0.9; // TODO tune

    public static final double indexingMechanismRotationsPerSec = 10;
    public static final int tunnelMotorId = 0;
    public static final double statorCurrentLimit = 60; // TODO
    public static final double supplyCurrentLimit = 40; // TODO
    public static final InvertedValue motorInvert =
        InvertedValue.Clockwise_Positive; // TODO set these
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double stoppedMechanismRotationsPerSec = 0.1; // TODO
    public static final double atSpeedMechanismRotationsPerSec =
        0.95 * indexingMechanismRotationsPerSec; // TODO
    public static final double motorToMechanismRatio = 1.5;
  }

  public static class Flywheel {
    public static final int motorId = 23;
    public static final int followerMotorId = 24;
    public static final double statorCurrentLimit = 60; // TODO
    public static final double supplyCurrentLimit = 40;
    public static final InvertedValue motorInvert = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double allowedVelocityErrorRPS = 5.0; // TODO

    public static final double motorToMechanismRatio = 1;
    public static final double idleMechanismRPS = 5;
    public static final double shootingMechanismRPS = 10;
    public static final int canandcolorId = 0;
    public static final double minFuelDetectionProximity = 0.2;
    public static final double allowedVelocityErrorMechanismRPS = 0.2;
  }

  public static class VisionObjectDetection {

    public static final Transform3d robotCenterToCamera = new Transform3d(); // TODO add
    public static final String hostname = null;
  }

  public static class Turret {
    public static final int motorId = 13; // TODO
    public static final double kS = 0; // TODO
    public static final double kV = 0; // TODO
    public static final double kP = 1; // TODO
    public static final double kI = 0; // TODO
    public static final double kD = 0; // TODO
    public static final double statorCurrentLimit = 60; // TODO
    public static final double supplyCurrentLimit = 40; // TODO
    public static final InvertedValue motorInvert = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final double zeroAzimuth = 0.0;
    public static final double tolerance = 0.0; // TODO
    public static final double physicalLimitDeg = 360.0; // TODO
    public static final double physicalLimitToleranceDeg = 5.0; // TODO
    public static final double CANCoderOneRatio = 3.0; // TODO
    public static final double CANCoderTwoRatio = 8.0; // TODO
    public static final double turretGearRatio = 90.0; // TODO
    public static final double minLimitAzimuth = -360.0; // TODO
    public static final double maxLimitAzimuth = 360.0; // TODO
    public static final double offsetAzimuth = 90.0; // TODO
  }

  public static class Hood {
    public static final int servoChannel = 12;
    public static final int encoderId = 1;
    public static final double gearRatio = 0.1;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double statorCurrentLimit = 60; // TODO
    public static final double supplyCurrentLimit = 40;
    public static final InvertedValue motorInvert = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final int idleVelocity = 0;
    public static final double hoodTolerance = 0.1;
    public static final double homingVelocity = -0.2;
  }

  public static class Control {
    public static final int toggle1ButtonNumber = 1; // TODO set these
  }

  public static class ShootingParameters {
    private final double flywheelRPS;
    private final double hoodAngleDeg;
    private final double timeOfFlightSec;

    public ShootingParameters(double flywheelRPS, double hoodAngleDeg, double timeOfFlightSec) {
      this.flywheelRPS = flywheelRPS;
      this.hoodAngleDeg = hoodAngleDeg;
      this.timeOfFlightSec = timeOfFlightSec;
    }

    public double getFlywheelRPS() {
      return flywheelRPS;
    }

    public double getHoodAngleDeg() {
      return hoodAngleDeg;
    }

    public double getTimeOfFlightSec() {
      return timeOfFlightSec;
    }
  }

  public static class ShootingManager {
    public static final InterpolatingTreeMap<Double, ShootingParameters> shooterMap =
        new InterpolatingTreeMap<Double, ShootingParameters>(
            null, null); // TODO not sure what to put for constructor

    static {
      // shooterMap.put()
    }

    // TODO figure these out
  }

  public static class ShootingTargetPoses {
    // Right/left are determined as view from alliance driver station

    public static class Red {
      public static final Pose2d hubPose = new Pose2d();
      public static final Pose2d allianceRightPose = new Pose2d();
      public static final Pose2d allianceLeftPose = new Pose2d();
      public static final Pose2d neutralRightPose = new Pose2d();
      public static final Pose2d neutralLeftPose = new Pose2d();
    }

    public static class Blue {
      public static final Pose2d hubPose = new Pose2d();
      public static final Pose2d allianceRightPose = new Pose2d();
      public static final Pose2d allianceLeftPose = new Pose2d();
      public static final Pose2d neutralRightPose = new Pose2d();
      public static final Pose2d neutralLeftPose = new Pose2d();
    }
  }
}
