package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.Region2d;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class DemoConfig {
  // All units in meters

  public static final boolean maxDriveSpeedOverride =
      true; // Set to true to override the normal speed (4.775 m/s)
  public static final LinearVelocity maxDriveSpeed = MetersPerSecond.of(0.7);
  public static final double rotationSpeedMultiplier = 1.5;

  public static final boolean useGeofence = true;
  public static final boolean manualZero =
      true; // Allows for the coast mode button to zero the robot, where it is placed against the
  // field border at 0,0 (bottom left corner), intake facing positive X

  public static boolean overrideShootingParams = false; // Does not apply to flywheel
  public static double tunnelRPS = 45 * 0.5;
  public static double spindexerRPS = 9 * 0.5;
  public static boolean useManualFlywheelLimit = false; // Likely cause robot to undershoot
  public static double flywheelLimitRPS = 20;

  // Field layout settings (in meters)
  // Layout guide at https://miro.com/app/board/uXjVHsdpnVk=/?share_link_id=345613555076 (requires
  // Clockwork login)
  public static int aprilTagAID = 21;
  public static int aprilTagBID = 22;

  public static boolean useCustomField = true;
  public static double demoFieldLength = Units.feetToMeters(15); // Long side,
  public static double demoFieldWidth =
      Units.feetToMeters(10); // Shorter side, should be side with an AprilTag

  public static boolean shootToOppositeSide = true;
  public static double shootingTargetOffset =
      Units.inchesToMeters(-24); // Positive is inside field, negative is outside field

  public static double aprilTagHorizontalOffset =
      Units.inchesToMeters(-12); // Offset from the long side of the field
  public static double aprilTagOtherOffset =
      Units.inchesToMeters(12); // Offset from the short side of the field yes i know scuffed name
  public static double aprilTagVerticalOffset = Units.inchesToMeters(39.875); // Offset from ground
  public static double aprilTagRotation = Math.PI / 4; // Rotation of the AprilTag in radians

  // Stuff that shouldn't change in between demos
  public static double robotMaxLength = 1.002129;
  public static double robotLength = 0.810666; // just robot, no intake
  public static double robotMaxWidth = 0.912241;
  public static double robotMaxSide = Math.max(robotMaxLength, robotMaxWidth);

  // Other classes/utils

  public class DemoFields {
    public static final double fieldLength = demoFieldLength;
    public static final double fieldWidth = demoFieldWidth;

    public static final double tolerance = 0.0;

    // Used for geofencing
    public static final double margin =
        0.6; // Distance from border to start slowing down; TODO tune this

    public static final double minX = robotMaxSide / 2;
    public static final double maxX = fieldLength - robotMaxSide / 2;
    public static final double minY = robotMaxSide / 2;
    public static final double maxY = fieldWidth - robotMaxSide / 2;

    // TODO check if these rotations are correct
    public static final AprilTagFieldLayout aprilTagFieldLayout =
        new AprilTagFieldLayout(
            List.of(
                new AprilTag(
                    aprilTagAID,
                    new Pose3d(
                        new Translation3d(
                            -aprilTagOtherOffset, aprilTagHorizontalOffset, aprilTagVerticalOffset),
                        new Rotation3d(0, 0, aprilTagRotation))),
                new AprilTag(
                    aprilTagBID,
                    new Pose3d(
                        new Translation3d(
                            fieldLength + aprilTagOtherOffset,
                            fieldWidth - aprilTagHorizontalOffset,
                            aprilTagVerticalOffset),
                        new Rotation3d(0, 0, aprilTagRotation + Math.PI)))),
            fieldLength,
            fieldWidth);

    public static final Translation2d leftTarget =
        new Translation2d(shootingTargetOffset, fieldWidth / 2);
    public static final Translation2d rightTarget =
        new Translation2d(fieldLength - shootingTargetOffset, fieldWidth / 2);
    public static final Translation2d centerTarget =
        new Translation2d(fieldLength / 2, fieldWidth / 2);

    public static final Region2d field =
        new Region2d(new Translation2d(0, 0), new Translation2d(fieldLength, fieldWidth), "field");
    public static final Region2d leftZone =
        new Region2d(
            new Translation2d(-tolerance, -tolerance),
            new Translation2d(fieldLength / 2, fieldWidth + tolerance),
            "leftZone");
    public static final Region2d rightZone =
        new Region2d(
            new Translation2d(fieldLength / 2, -tolerance),
            new Translation2d(fieldLength + tolerance, fieldWidth + tolerance),
            "rightZone");
    public static final Region2d allowedArea =
        new Region2d(new Translation2d(minX, minY), new Translation2d(maxX, maxY), "allowedArea");
    public static final Region2d marginArea =
        new Region2d(
            new Translation2d(minX + margin, minY + margin),
            new Translation2d(maxX - margin, maxY - margin),
            "marginArea");

    public static void log() {
      leftZone.logPoints();
      rightZone.logPoints();
      field.logPoints();
      allowedArea.logPoints();
      marginArea.logPoints();
      Logger.recordOutput(
          "DemoFields/aprilTagA", aprilTagFieldLayout.getTagPose(aprilTagAID).orElse(null));
      Logger.recordOutput(
          "DemoFields/aprilTagB", aprilTagFieldLayout.getTagPose(aprilTagBID).orElse(null));
    }
  }
}
