package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.subsystems.shooter.areaManager.AreaManager.Zone;

public class ShootFixed extends Command {
  private Shooter shooter;
  private Drive drive;

  public ShootFixed(Shooter shooter) {
    this.shooter = shooter;
  }

  @Override
  public void execute() {
    if (!AreaManager.isShootingArea(drive.getRobotPose().getTranslation())) {
      shooter.requestIdle();
    } else {
      if (AreaManager.getZoneOfPosition(drive.getRobotPose().getTranslation())
          == Zone.ALLIANCE_ZONE) {
        shooter.requestShoot(true, true);
      } else {
        shooter.requestShoot(true, false);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.requestIdle();
  }
}
