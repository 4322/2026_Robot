package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.subsystems.shooter.areaManager.AreaManager.Zone;

public class Shoot extends Command {
  private Shooter shooter;
  private Drive drive;
  private boolean ignoreArea;
  private Shooter.fixedAreaPlacement area;

  public Shoot(Shooter shooter, Drive drive, Shooter.fixedAreaPlacement area) {
    this.shooter = shooter;
    this.drive = drive;
    this.area = area;
    this.ignoreArea = false;
  }

  public Shoot(Shooter shooter, Drive drive, boolean ignoreArea, Shooter.fixedAreaPlacement area) {
    this.shooter = shooter;
    this.drive = drive;
    this.ignoreArea = ignoreArea;
    this.area = area;
  }

  @Override
  public void execute() {
    if ((!AreaManager.isShootingArea(drive.getTurretTranslation())
            || AreaManager.isTrench(drive.getTurretTranslation()))
        && !ignoreArea) {
      shooter.requestIdle(null, null);
    } else {
      if (AreaManager.getZoneOfPosition(drive.getTurretTranslation()) == Zone.ALLIANCE_ZONE
          || AreaManager.isTrench(drive.getTurretTranslation())) {
        shooter.requestShoot(false, true, area);
      } else {
        shooter.requestShoot(false, false, area);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.requestIdle(null, null);
  }
}
