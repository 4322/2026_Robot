package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.hood.Hood;
import org.littletonrobotics.junction.Logger;

public class ShooterCommands {

  public static Command shoot(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.requestShoot(false);
          Logger.recordOutput("Shooter/command", "shoot");
        },
        shooter);
  }

  public static Command shootFixed(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.requestShoot(true);
          Logger.recordOutput("Shooter/command", "shootFixed");
        },
        shooter);
  }

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
    return Commands.run(
        () -> {
          shooter.requestIdle();
          Logger.recordOutput("Shooter/command", "idle");
        },
        shooter);
  }

  public static Command stop(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.requestStop();
          Logger.recordOutput("Shooter/command", "stop");
        },
        shooter);
  }

  public static Command trenchOverride(Hood hood) {
    return Commands.run(
            () -> {
              hood.trenchOverride(true);
              Logger.recordOutput("Shooter/command", "trenchOverride");
            })
        .finallyDo(() -> hood.trenchOverride(false));
  }

  public static Command unjam(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.requestUnjam();
          Logger.recordOutput("Shooter/command", "unjam");
        },
        shooter);
  }

  public static Command setAutoShoot(Shooter shooter, boolean enabled) {
    return Commands.run(
        () -> {
          shooter.setAutoShoot(enabled);
          Logger.recordOutput("Shooter/command", "toggleAutoShoot");
        });
  }
}
