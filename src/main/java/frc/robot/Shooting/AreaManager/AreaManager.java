package frc.robot.Shooting.AreaManager;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Drive.Drive;

public class AreaManager {
  private Drive drive;

  public enum Zone {
    NONE,
    RED_ALLIANCE,
    BLUE_ALLIANCE,
    LEFT_NEUTRAL,
    RIGHT_NEUTRAL
  }

  public Zone zone = Zone.NONE;

  public AreaManager(Drive drive) {
    this.drive = drive;
    // // determine alliance and set zone
    // DriverStation.Alliance alliance = DriverStation.getAlliance();
    // if (alliance == DriverStation.Alliance.Red) {
    //   this.zone = Zone.RED_ALLIANCE;
    // } else if (alliance == DriverStation.Alliance.Blue) {
    //   this.zone = Zone.BLUE_ALLIANCE;
    // }
  }

  // Red trenches
  Rectangle2d trenchLeftRed =
      new Rectangle2d(
          new Translation2d(0, 0),
          new Translation2d(0, 0)); // TODO: Fill in with actual coordinates

  Rectangle2d trenchRightRed =
      new Rectangle2d(
          new Translation2d(0, 0),
          new Translation2d(0, 0)); // TODO: Fill in with actual coordinates

  // Alliance areas
  Rectangle2d redAllianceZone =
      new Rectangle2d(
          new Translation2d(0, 0),
          new Translation2d(0, 0)); // TODO: Fill in with actual coordinates

  Rectangle2d blueAllianceZone =
      new Rectangle2d(
          new Translation2d(0, 0),
          new Translation2d(0, 0)); // TODO: Fill in with actual coordinates

  Rectangle2d leftNeutralZone =
      new Rectangle2d(
          new Translation2d(0, 0),
          new Translation2d(0, 0)); // TODO: Fill in with actual coordinates

  Rectangle2d rightNeutralZone =
      new Rectangle2d(
          new Translation2d(0, 0),
          new Translation2d(0, 0)); // TODO: Fill in with actual coordinates

  public boolean isShootingArea() {
    Translation2d drivePosition = drive.getPose().getTranslation();
    return !(trenchLeftRed.contains(drivePosition) || trenchRightRed.contains(drivePosition));
  }

  public Zone getZone() {
    return zone;
  }
}
