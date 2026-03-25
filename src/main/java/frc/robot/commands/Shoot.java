package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.subsystems.shooter.areaManager.AreaManager.Zone;

public class Shoot extends Command {
  private Shooter shooter;
  private Drive drive;

  public Shoot(Shooter shooter) {
    this.shooter = shooter;
  }

  @Override
  public void execute() {
    if (!AreaManager.isShootingArea(drive.getTurretTranslation()) || AreaManager.isTrench(drive.getTurretTranslation())) {
      shooter.requestIdle();
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
    shooter.requestIdle();
  }
}
