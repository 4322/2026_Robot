package frc.robot.Shooting.AreaManager;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Drive.Drive;

public class AreaManager {
  private Drive drive;

  private AreaManager(Drive drive) {
    this.drive = drive;
  }

  Translation2d drivePosition = drive.getPose().getTranslation();

  // Red
  Rectangle2d trenchLeftRed = new Rectangle2d(new Translation2d(0, 0), new Translation2d(0, 0));

  Rectangle2d trenchRightRed = new Rectangle2d(new Translation2d(0, 0), new Translation2d(0, 0));

  public boolean inNoShootingArea() {
    return trenchLeftRed.contains(drivePosition) || trenchRightRed.contains(drivePosition);
  }
}
