package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.Constants.FiringParameters;

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
    public static final double goalToleranceDeg = 1.0; // TODO
    public static final double CANCoderOneRatio = 3.0; // TODO
    public static final double CANCoderTwoRatio = 8.0; // TODO
    public static final double CANCoderOneOffset = 0.0; // TODO
    public static final double CANCoderTwoOffset = 0.0; // TODO
    public static final double turretGearRatio = 90.0; // TODO
    public static final double minPhysicalLimitDeg = -360.0; // TODO
    public static final double maxPhysicalLimitDeg = 360.0; // TODO
    public static final double midPointPhysicalDeg =
        (minPhysicalLimitDeg + maxPhysicalLimitDeg) / 2.0;
    public static final double unwindToleranceDeg = 10.0; // TODO
    public static final double minUnwindLimitDeg = minPhysicalLimitDeg + unwindToleranceDeg; // TODO
    public static final double maxUnwindLimitDeg = maxPhysicalLimitDeg - unwindToleranceDeg; // TODO
  }

  public static class Hood {
    public static final int servoHubId = 10;
    public static final int servoChannelId = 1;
    public static final int servoDefaultPWM = 0;
  }

  public static class Control {
    public static final int toggle1ButtonNumber = 1; // TODO set these
  }

  public static class FiringParameters {
    private final double flywheelRPM;
    private final double hoodAngleDeg;
    private final double timeOfFlightSec;

    public FiringParameters(double flywheelRPM, double hoodAngleDeg, double timeOfFlightSec) {
      this.flywheelRPM = flywheelRPM;
      this.hoodAngleDeg = hoodAngleDeg;
      this.timeOfFlightSec = timeOfFlightSec;
    }

    public double getFlywheelRPM() {
      return flywheelRPM;
    }

    public double getHoodAngleDeg() {
      return hoodAngleDeg;
    }

    public double getTimeOfFlightSec() {
      return timeOfFlightSec;
    }

    public static FiringParameters interpolate(
        FiringParameters start, FiringParameters end, double howFar) {
      return new FiringParameters(
          start.flywheelRPM + (end.flywheelRPM - start.flywheelRPM) * howFar,
          start.hoodAngleDeg + (end.hoodAngleDeg - start.hoodAngleDeg) * howFar,
          start.timeOfFlightSec + (end.timeOfFlightSec - start.timeOfFlightSec) * howFar);
    }
  }

  public static class FiringManager {
    public static final InterpolatingTreeMap<Double, FiringParameters> firingMap =
        new InterpolatingTreeMap<Double, FiringParameters>(
            InverseInterpolator.forDouble(), FiringParameters::interpolate);

    // Reverse map for velocity to distance lookup
    public static final InterpolatingDoubleTreeMap velocityToDistanceMap =
        new InterpolatingDoubleTreeMap();

    public static final double latencyCompensation = 0;

    // Add entry to both maps
    public static void putShooterEntry(double distance, FiringParameters params) {
      firingMap.put(distance, params);
      double velocity = distance / params.getTimeOfFlightSec();
      velocityToDistanceMap.put(velocity, distance);
    }

    static { // TODO tuning points will go here
      // putShooterEntry(distance, new ShootingParameters(rpm, hoodDeg, tofSec));
    }
  }

  public static class FiringTargetTranslations {
    // Right/left are determined as view from alliance driver station

    public static class Red {
      public static final Translation2d hubTranslation = new Translation2d();
      public static final Translation2d allianceRightTranslation = new Translation2d();
      public static final Translation2d allianceLeftTranslation = new Translation2d();
      public static final Translation2d neutralRightTranslation = new Translation2d();
      public static final Translation2d neutralLeftTranslation = new Translation2d();
    }

    public static class Blue {
      public static final Translation2d hubTranslation = new Translation2d();
      public static final Translation2d allianceRightTranslation = new Translation2d();
      public static final Translation2d allianceLeftTranslation = new Translation2d();
      public static final Translation2d neutralRightTranslation = new Translation2d();
      public static final Translation2d neutralLeftTranslation = new Translation2d();
    }
  }
}
