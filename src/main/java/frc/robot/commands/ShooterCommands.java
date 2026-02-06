package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import java.util.function.BooleanSupplier;

public class ShooterCommands {

  public static Command shoot(Shooter shooter, BooleanSupplier end) {
    BooleanSupplier mechanismsAtSpeed = () -> shooter.isMechanismsAtSpeed();
    BooleanSupplier hoodAtAngle = () -> shooter.isHoodAtAngle();
    BooleanSupplier flywheelAtSpeed = () -> shooter.isFlywheelAtSpeed();

    return Commands.run(() -> shooter.setState(ShooterState.PRESHOOT), shooter)
        .until(flywheelAtSpeed)
        .andThen(
            Commands.run(
                () -> {
                  if (shooter.needsToUnwind()) {
                    shooter.setState(ShooterState.UNWIND);
                  } else {
                    shooter.setState(ShooterState.SHOOT);
                  }
                },
                shooter))
        .until(end);
  }

  // Main commands

  public static Command inhibitAutoShoot(Shooter shooter, BooleanSupplier toggleOn) {
    BooleanSupplier end = () -> !toggleOn.getAsBoolean();
    BooleanSupplier needsToUnwind = () -> shooter.needsToUnwind();
    BooleanSupplier unwinded = () -> shooter.isUnwinded();
    BooleanSupplier unwindComplete = () -> shooter.isUnwindComplete();

    return Commands.run(
            () -> {
              if ((!unwinded.getAsBoolean() || needsToUnwind.getAsBoolean())
                  && !unwindComplete.getAsBoolean()) {
                shooter.setState(ShooterState.UNWIND);
              } else {
                shooter.setState(ShooterState.IDLE);
              }
            })
        .until(end); // BooleanSupplier tied to operator toggle 1 being turned off
  }

  public static Command areaInhibitAutoShoot(Shooter shooter, Drive drive) {
    BooleanSupplier needsToUnwind = () -> shooter.needsToUnwind();
    BooleanSupplier unwinded = () -> shooter.isUnwinded();
    BooleanSupplier unwindComplete = () -> shooter.isUnwindComplete();
    BooleanSupplier inShootingArea =
        () -> AreaManager.isShootingArea(drive.getPose().getTranslation());

    return Commands.run(
            () -> {
              if ((!unwinded.getAsBoolean() || needsToUnwind.getAsBoolean())
                  && !unwindComplete.getAsBoolean()) {
                shooter.setState(ShooterState.UNWIND);
              } else {
                shooter.setState(ShooterState.IDLE);
              }
            })
        .until(inShootingArea);
  }

  // Default command
  public static Command autoShoot(Shooter shooter, Drive drive) {
    BooleanSupplier needsToUnwind = () -> shooter.needsToUnwind();

    BooleanSupplier end = () -> false;

    return shoot(shooter, end);
  }
}
