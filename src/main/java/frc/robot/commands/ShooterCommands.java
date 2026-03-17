package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.hood.Hood;

public class ShooterCommands {

  public static Command shoot(Shooter shooter) {

    return Commands.run(() -> shooter.requestShoot(), shooter);
  }

  // Main commands

  public static Command aimAndShoot(Shooter shooter, Drive drive) {
    return Commands.parallel(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -RobotContainer.controller.getLeftY(),
            () -> -RobotContainer.controller.getLeftX(),
            () -> Rotation2d.fromDegrees(shooter.getTargetTurretAngleDeg()),
            Constants.Turret.originToTurret),
        shoot(shooter));
  }

  public static Command idle(Shooter shooter) {
    return Commands.run(() -> shooter.requestIdle(), shooter).finallyDo(() -> shooter.endIdle());
  }

  public static Command trenchOverride(Hood hood) {
    return Commands.run(() -> hood.trenchOverride(true))
        .finallyDo(() -> hood.trenchOverride(false));
  }

  public static Command unjam(Shooter shooter) {
    return Commands.run(() -> shooter.requestUnjam(), shooter);
  }
}
