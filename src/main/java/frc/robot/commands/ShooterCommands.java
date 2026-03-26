package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class ShooterCommands {

  public static Command autoShoot(Shooter shooter, Drive drive, Intake intake) {
    return new Shoot(shooter, drive).onlyIf(() -> intake.hasExtended());
  }

  // Used in auto
  public static Command autoShootNoAreaCheck(Shooter shooter, Drive drive, Intake intake) {
    return new Shoot(shooter, drive, true).onlyIf(() -> intake.hasExtended());
  }

  public static Command fixedShoot(Shooter shooter, Drive drive, Intake intake) {
    return new ShootFixed(shooter).onlyIf(() -> intake.hasExtended());
  }

  public static Command aimAndShoot(Shooter shooter, Drive drive, Intake intake) {
    return Commands.parallel(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -RobotContainer.controller.getLeftY(),
            () -> -RobotContainer.controller.getLeftX(),
            () -> Rotation2d.fromDegrees(shooter.getTargetTurretAngleDeg()),
            Constants.Turret.originToTurret),
        autoShoot(shooter, drive, intake));
  }

  public static Command idle(Shooter shooter, Intake intake) {
    return Commands.run(
            () -> {
              shooter.requestIdle();
              Logger.recordOutput("Shooter/command", "idle");
            },
            shooter)
        .onlyIf(() -> intake.hasExtended());
  }

  public static Command stop(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.requestStop();
          Logger.recordOutput("Shooter/command", "stop");
        },
        shooter);
  }

  public static Command trenchOverride(Shooter shooter) {
    return Commands.run(
            () -> {
              shooter.trenchOverride(true);
              Logger.recordOutput("Shooter/command", "trenchOverride");
            })
        .finallyDo(() -> shooter.trenchOverride(false));
  }

  public static Command unjam(Shooter shooter) {
    return Commands.run(
            () -> {
              shooter.unjamOverride(true);
              Logger.recordOutput("Shooter/command", "unjam");
            },
            shooter)
        .finallyDo(() -> shooter.unjamOverride(false));
  }
}
