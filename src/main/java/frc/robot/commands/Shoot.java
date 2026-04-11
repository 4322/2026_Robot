package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.subsystems.shooter.areaManager.AreaManager.Zone;

public class Shoot extends Command {
  private Shooter shooter;
  private Drive drive;
  private boolean ignoreArea;

  public Shoot(Shooter shooter, Drive drive) {
    this.shooter = shooter;
    this.drive = drive;
    this.ignoreArea = false;
  }

  public Shoot(Shooter shooter, Drive drive, boolean ignoreArea) {
    this.shooter = shooter;
    this.drive = drive;
    this.ignoreArea = ignoreArea;
  }

  @Override
  public void execute() {
    Logger.recordOutput("Shooter/shooterCommand/currentZone", AreaManager.getZoneOfPosition(drive.getTurretTranslation()).toString());
    if ((!AreaManager.isShootingArea(drive.getTurretTranslation())) && !ignoreArea) {
      shooter.requestIdle(null, null, null);
    } else {
      if (AreaManager.getZoneOfPosition(drive.getTurretTranslation()) == Zone.ALLIANCE_ZONE) {
        shooter.requestShoot(false, true);
      } else {
        shooter.requestShoot(false, false);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.requestIdle(null, null, null);
  }
}
