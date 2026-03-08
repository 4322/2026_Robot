package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {

  public static Command shoot(Shooter shooter) {

    return Commands.run(() -> shooter.requestShoot(), shooter);
  }

  // Main commands
  

  public static Command aimAndShoot(Shooter shooter, Drive drive) {
    return Commands.parallel(DriveCommands.driveAzimuthRotate(drive), shoot(shooter));
  }

  public static Command idle(Shooter shooter) {
    return Commands.run(() -> shooter.requestIdle(), shooter);
  }
}
