package frc.robot.subsystems.shooter.areaManager;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;

public class AreaManager {
  // TODO: Put IN field dimensions

  // All dimensions are in meters
  // Alliance Zones
  private static Rectangle2d redAllianceZone =
      new Rectangle2d(
          new Translation2d( Units.inchesToMeters(651.22), Units.inchesToMeters(317.69)),
          new Translation2d(Units.inchesToMeters(651.22), 0));
  private static Rectangle2d blueAllianceZone =
      new Rectangle2d(
          new Translation2d(0, Units.inchesToMeters(317.69)),
          new Translation2d(Units.inchesToMeters(158.61), 0));

  // Neutral Zones
  private static Rectangle2d leftNeutralZone =
      new Rectangle2d(
          new Translation2d(4.03, Units.inchesToMeters(317.69)),
          new Translation2d(
              11.22, Units.inchesToMeters(158.84))); // When looking from Blue Alliance side
  private static Rectangle2d rightNeutralZone =
      new Rectangle2d(
          new Translation2d(4.03, Units.inchesToMeters(158.84)),
          new Translation2d(11.22, 0)); // When looking from Blue Alliance side
  // Opposition Zones
  private static Rectangle2d leftBlueOppositionZone =
      new Rectangle2d(
          new Translation2d(0, Units.inchesToMeters(317.69)),
          new Translation2d(Units.inchesToMeters(158.61), Units.inchesToMeters(158.84)));
  private static Rectangle2d rightBlueOppositionZone =
      new Rectangle2d(
          new Translation2d(0, Units.inchesToMeters(158.84)),
          new Translation2d(Units.inchesToMeters(158.61), 0));
  private static Rectangle2d leftRedOppositionZone =
      new Rectangle2d(
          new Translation2d(12.51, Units.inchesToMeters(317.69)),
          new Translation2d(Units.inchesToMeters(651.22), Units.inchesToMeters(158.84)));
  private static Rectangle2d rightRedOppositionZone =
      new Rectangle2d(
          new Translation2d(12.51, Units.inchesToMeters(158.84)),
          new Translation2d(Units.inchesToMeters(651.22), 0));

  // Non-Shooting Areas
  private static Rectangle2d trenchLeftRed =
      new Rectangle2d(
          new Translation2d(
              (Units.inchesToMeters(47) - Units.inchesToMeters(12)), Units.inchesToMeters(317.69)),
          new Translation2d(
              (Units.inchesToMeters(47) + Units.inchesToMeters(12)), Units.inchesToMeters(255.1)));
  private static Rectangle2d trenchRightRed =
      new Rectangle2d(
          new Translation2d(
              (Units.inchesToMeters(47) - Units.inchesToMeters(12)), Units.inchesToMeters(62.59)),
          new Translation2d((Units.inchesToMeters(47) + Units.inchesToMeters(12)), 0));

  private static Rectangle2d bumpLeftRed =
      new Rectangle2d(new Translation2d(0, 6.1722), new Translation2d(2.0574, 10.287));
  private static Rectangle2d bumpRightRed =
      new Rectangle2d(new Translation2d(0, 10.287), new Translation2d(2.0574, 14.4018));
  private static Rectangle2d backOfHubRed =
      new Rectangle2d(new Translation2d(0, 12.3444), new Translation2d(8.2296, 16.4592));
  private static Rectangle2d frontOfHubRed =
      new Rectangle2d(new Translation2d(16.4592, 0), new Translation2d(24.6888, 4.1148));

  // Blue Alliance Non-Shooting Areas
  private static Rectangle2d trenchLeftBlue =
      new Rectangle2d(new Translation2d(0, 4.1148), new Translation2d(8.2296, 8.2296));
  private static Rectangle2d trenchRightBlue =
      new Rectangle2d(new Translation2d(0, 8.2296), new Translation2d(8.2296, 12.3444));
  private static Rectangle2d bumpLeftBlue =
      new Rectangle2d(new Translation2d(0, 6.1722), new Translation2d(2.0574, 10.287));
  private static Rectangle2d bumpRightBlue =
      new Rectangle2d(new Translation2d(0, 10.287), new Translation2d(2.0574, 14.4018));
  private static Rectangle2d backOfHubBlue =
      new Rectangle2d(new Translation2d(0, 12.3444), new Translation2d(8.2296, 16.4592));
  private static Rectangle2d frontOfHubBlue =
      new Rectangle2d(new Translation2d(16.4592, 0), new Translation2d(24.6888, 4.1148));

  public enum Zone {
    ALLIANCE_ZONE,
    LEFT_OPPOSITION,
    RIGHT_OPPOSITION,
    LEFT_NEUTRAL,
    RIGHT_NEUTRAL
  }

  private static Zone zone = Zone.ALLIANCE_ZONE;

  public static boolean isShootingArea(Translation2d position) {
    return !((trenchLeftRed.contains(position)
            || trenchRightRed.contains(position)
            || bumpLeftRed.contains(position)
            || bumpRightRed.contains(position)
            || trenchLeftBlue.contains(position)
            || trenchRightBlue.contains(position)
            || bumpLeftBlue.contains(position)
            || bumpRightBlue.contains(position)
            || backOfHubRed.contains(position)
            || backOfHubBlue.contains(position))
        || (Robot.alliance == Alliance.Red
            ? (frontOfHubRed.contains(position))
            : (frontOfHubBlue.contains(position))));
  }

  public static Zone getZoneOfPosition(Translation2d position) {
    if (Robot.alliance == Alliance.Red) {
      if (redAllianceZone.contains(position)) {
        zone = Zone.ALLIANCE_ZONE;
      } else if (leftBlueOppositionZone.contains(position)) {
        zone = Zone.LEFT_OPPOSITION;
      } else if (rightBlueOppositionZone.contains(position)) {
        zone = Zone.RIGHT_OPPOSITION;
      } else if (leftNeutralZone.contains(position)) {
        zone = Zone.RIGHT_NEUTRAL;
      } else if (rightNeutralZone.contains(position)) {
        zone = Zone.LEFT_NEUTRAL;
      }
    } else {
      if (blueAllianceZone.contains(position)) {
        zone = Zone.ALLIANCE_ZONE;
      } else if (leftRedOppositionZone.contains(position)) {
        zone = Zone.LEFT_OPPOSITION;
      } else if (rightRedOppositionZone.contains(position)) {
        zone = Zone.RIGHT_OPPOSITION;
      } else if (leftNeutralZone.contains(position)) {
        zone = Zone.LEFT_NEUTRAL;
      } else if (rightNeutralZone.contains(position)) {
        zone = Zone.RIGHT_NEUTRAL;
      }
    }
    return zone;
  }
}
