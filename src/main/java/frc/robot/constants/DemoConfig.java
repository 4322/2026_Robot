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

// Todo list for Ryan before Saturday
/*
- Low powered shallow shooting
*/

public class DemoConfig {
  // All units in meters

  public static final boolean maxDriveSpeedOverride =
      true; // Set to true to override the normal speed (4.775 m/s)
  public static final LinearVelocity maxDriveSpeed = MetersPerSecond.of(1);

  public static final boolean useGeofence = true;
  public static final boolean manualZero = true; // Allows for the coast mode button to zero the robot, where it is placed against the field border at 0,0 (bottom left corner), intake facing positive X

  // Field layout settings (in meters)
  // Layout guide at https://miro.com/app/board/uXjVHsdpnVk=/?share_link_id=345613555076 (requires
  // Clockwork login)
  public static int aprilTagAID = 1;
  public static int aprilTagBID = 2;

  public static boolean useCustomField = true;
  public static double demoFieldLength = Units.feetToMeters(20); // Long side,
  public static double demoFieldWidth =
      Units.feetToMeters(10); // Shorter side, should be side with an AprilTag

    public static double shootingTargetOffset = Units.feetToMeters(1); // Positive is inside field, negative is outside field

  public static double aprilTagHorizontalOffset =
      Units.inchesToMeters(24); // Offset from the long side of the field
  public static double aprilTagVerticalOffset = Units.inchesToMeters(24); // Offset from ground

  // Stuff that shouldn't change in between demos
  public static double robotMaxLength = 0; // TODO; measure with intake out
  public static double robotLength = 0; // just robot, no intake
  public static double robotMaxWidth = 0;
  public static double robotMaxSide = Math.max(robotMaxLength, robotMaxWidth);

  // Other classes/utils

  public class DemoFields {
    public static final double fieldLength = demoFieldLength;
    public static final double fieldWidth = demoFieldWidth;

    public static final double tolerance = 10.0;

    // Used for geofencing
    public static final double margin = 0.8; // TODO tune this

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
                        new Translation3d(aprilTagHorizontalOffset, 0, aprilTagVerticalOffset),
                        new Rotation3d(Units.degreesToRadians(-90), 0, 0))),
                new AprilTag(
                    aprilTagBID,
                    new Pose3d(
                        new Translation3d(
                            fieldWidth - aprilTagHorizontalOffset,
                            fieldLength,
                            aprilTagVerticalOffset),
                        new Rotation3d(Units.degreesToRadians(90), 0, 0)))),
            fieldLength,
            fieldWidth);
    
    public static final Translation2d leftTarget = new Translation2d(shootingTargetOffset, aprilTagHorizontalOffset);
    public static final Translation2d rightTarget = new Translation2d(fieldLength - shootingTargetOffset, fieldWidth - aprilTagHorizontalOffset);
    public static final Translation2d centerTarget = new Translation2d(fieldLength / 2, fieldWidth / 2);

    public static final Region2d leftZone = new Region2d(new Translation2d(-tolerance,-tolerance), new Translation2d(fieldLength / 2, fieldWidth), "leftZone");
    public static final Region2d rightZone = new Region2d(new Translation2d(fieldLength / 2, 0), new Translation2d(fieldLength + tolerance, fieldWidth + tolerance), "rightZone");
  }
}
