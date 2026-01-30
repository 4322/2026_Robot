package frc.robot.subsystems.shooter.areaManager;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;

public class AreaManager {
  private Drive drive;
  private Rectangle2d redAllianceZone;
  private Rectangle2d leftBlueOppositionZone;
  private Rectangle2d rightBlueOppositionZone;
  private Rectangle2d leftNeutralZone;
  private Rectangle2d rightNeutralZone;
  private Rectangle2d blueAllianceZone;
  private Rectangle2d leftRedOppositionZone;
  private Rectangle2d rightRedOppositionZone;
  private Rectangle2d trenchLeftRed;
  private Rectangle2d trenchRightRed;
  private Rectangle2d bumpLeftRed;
  private Rectangle2d bumpRightRed;
  private Rectangle2d trenchLeftBlue;
  private Rectangle2d trenchRightBlue;
  private Rectangle2d bumpLeftBlue;
  private Rectangle2d bumpRightBlue;
  private Rectangle2d backOfHubRed;
  private Rectangle2d backOfHubBlue;
  private Rectangle2d frontOfHubRed;
  private Rectangle2d frontOfHubBlue;

  public enum Zone {
    ALLIANCE_ZONE,
    LEFT_OPPOSITION,
    RIGHT_OPPOSITION,
    LEFT_NEUTRAL,
    RIGHT_NEUTRAL
  }

  private Zone zone = Zone.ALLIANCE_ZONE;
  private Zone objectZone = Zone.ALLIANCE_ZONE;

  public AreaManager(Drive drive) {
    this.drive = drive;
    // Red trenches

  }

  public void initializeZones() {
    // TODO: Put IN field dimensions

    // Red Alliance Zones
    redAllianceZone = new Rectangle2d(new Translation2d(0, 0), new Translation2d(0, 0));
    leftBlueOppositionZone = new Rectangle2d(new Translation2d(0, 0), new Translation2d(0, 0));
    rightBlueOppositionZone = new Rectangle2d(new Translation2d(0, 0), new Translation2d(0, 0));
    // Neutral Zones
    leftNeutralZone =
        new Rectangle2d(
            new Translation2d(0, 0),
            new Translation2d(0, 8.2296)); // When looking from Red Alliance side
    rightNeutralZone =
        new Rectangle2d(
            new Translation2d(0, 8.2296),
            new Translation2d(0, 16.4592)); // When looking from Red Alliance side
    // Blue Alliance Zones
    blueAllianceZone =
        new Rectangle2d(new Translation2d(16.4592, 0), new Translation2d(24.6888, 16.4592));
    leftRedOppositionZone =
        new Rectangle2d(new Translation2d(8.2296, 0), new Translation2d(16.4592, 8.2296));
    rightRedOppositionZone =
        new Rectangle2d(new Translation2d(8.2296, 8.2296), new Translation2d(16.4592, 16.4592));
    // Non-Shooting Areas
    trenchLeftRed =
        new Rectangle2d(new Translation2d(0, 4.1148), new Translation2d(8.2296, 8.2296));
    trenchRightRed =
        new Rectangle2d(new Translation2d(0, 8.2296), new Translation2d(8.2296, 12.3444));
    bumpLeftRed = new Rectangle2d(new Translation2d(0, 6.1722), new Translation2d(2.0574, 10.287));
    bumpRightRed =
        new Rectangle2d(new Translation2d(0, 10.287), new Translation2d(2.0574, 14.4018));
    backOfHubRed =
        new Rectangle2d(new Translation2d(0, 12.3444), new Translation2d(8.2296, 16.4592));
    frontOfHubRed =
        new Rectangle2d(new Translation2d(16.4592, 0), new Translation2d(24.6888, 4.1148));

    // Blue Alliance Non-Shooting Areas
    trenchLeftBlue =
        new Rectangle2d(new Translation2d(0, 4.1148), new Translation2d(8.2296, 8.2296));
    trenchRightBlue =
        new Rectangle2d(new Translation2d(0, 8.2296), new Translation2d(8.2296, 12.3444));
    bumpLeftBlue = new Rectangle2d(new Translation2d(0, 6.1722), new Translation2d(2.0574, 10.287));
    bumpRightBlue =
        new Rectangle2d(new Translation2d(0, 10.287), new Translation2d(2.0574, 14.4018));
    backOfHubBlue =
        new Rectangle2d(new Translation2d(0, 12.3444), new Translation2d(8.2296, 16.4592));
    frontOfHubBlue =
        new Rectangle2d(new Translation2d(16.4592, 0), new Translation2d(24.6888, 4.1148));
  }

  public void periodic() {
    Translation2d drivePosition = drive.getPose().getTranslation();
    getZoneOfObject(drivePosition);
  }

  public boolean isShootingArea() {
    Translation2d drivePosition = drive.getPose().getTranslation();
    return !((trenchLeftRed.contains(drivePosition)
            || trenchRightRed.contains(drivePosition)
            || bumpLeftRed.contains(drivePosition)
            || bumpRightRed.contains(drivePosition)
            || trenchLeftBlue.contains(drivePosition)
            || trenchRightBlue.contains(drivePosition)
            || bumpLeftBlue.contains(drivePosition)
            || bumpRightBlue.contains(drivePosition)
            || backOfHubRed.contains(drivePosition)
            || backOfHubBlue.contains(drivePosition))
        || (Robot.alliance == Alliance.Red
            ? (frontOfHubRed.contains(drivePosition))
            : (frontOfHubBlue.contains(drivePosition))));
  }

  public Zone getZone() {
    return zone;
  }

  public Zone getZoneOfObject(Translation2d position) {
    if (Robot.alliance == Alliance.Red) {
      if (redAllianceZone.contains(position)) {
        objectZone = Zone.ALLIANCE_ZONE;
      } else if (leftBlueOppositionZone.contains(position)) {
        objectZone = Zone.LEFT_OPPOSITION;
      } else if (rightBlueOppositionZone.contains(position)) {
        objectZone = Zone.RIGHT_OPPOSITION;
      } else if (leftNeutralZone.contains(position)) {
        objectZone = Zone.LEFT_NEUTRAL;
      } else if (rightNeutralZone.contains(position)) {
        objectZone = Zone.RIGHT_NEUTRAL;
      }
    } else {
      if (blueAllianceZone.contains(position)) {
        objectZone = Zone.ALLIANCE_ZONE;
      } else if (leftRedOppositionZone.contains(position)) {
        objectZone = Zone.LEFT_OPPOSITION;
      } else if (rightRedOppositionZone.contains(position)) {
        objectZone = Zone.RIGHT_OPPOSITION;
      } else if (leftNeutralZone.contains(position)) {
        objectZone = Zone.RIGHT_NEUTRAL;
      } else if (rightNeutralZone.contains(position)) {
        objectZone = Zone.LEFT_NEUTRAL;
      }
    }
    return objectZone;
  }
}
