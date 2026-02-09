package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import java.util.function.BooleanSupplier;

public class ShooterCommand {

  //   public static Command turretUnwind(Shooter shooter) {
  //  BooleanSupplier rewindComplete = () -> shooter.isRewindComplete();
  //     return Commands.run(() -> shooter.setState(ShooterState.UNWIND),
  // shooter).until(rewindComplete);
  //   }

  public static Command shoot(Shooter shooter, BooleanSupplier end) {
    BooleanSupplier mechanismsAtSpeed = () -> shooter.isMechanismsAtSpeed();
    BooleanSupplier hoodAtAngle = () -> shooter.isHoodAtAngle();
    BooleanSupplier flywheelAtSpeed = () -> shooter.isFlywheelAtSpeed();

    return Commands.run(
            () -> {
              shooter.setState(ShooterState.PRESHOOT);
              if (flywheelAtSpeed
                  .getAsBoolean()) { // TODO checks flywheel speed twice; check hood/turret
                shooter.setState(
                    ShooterState.SHOOT); // TODO only do this once; don't set again to preshoot
              }
            },
            shooter)
        .until(end);
  }

  public static Command idle(Shooter shooter) {
    return new InstantCommand(() -> shooter.setState(ShooterState.IDLE), shooter);
  }

  // Main commands

  public static Command inhibitAutoShoot(Shooter shooter, BooleanSupplier end) {
    return Commands.run(
            () -> {
              shooter.setState(ShooterState.IDLE);
            },
            shooter)
        .until(end); // BooleanSupplier will be somehow tied to operator toggle 1 being turned off
  }

  public static Command areaInhibitAutoShoot(Shooter shooter, Drive drive) { // TODO unwind
    BooleanSupplier needsToUnwind = () -> shooter.needsToUnwind();
    BooleanSupplier inShootingArea =
        () -> AreaManager.isShootingArea(drive.getPose().getTranslation());
    BooleanSupplier end = () -> needsToUnwind.getAsBoolean() || inShootingArea.getAsBoolean();
    return Commands.run(
            () -> {
              shooter.setState(ShooterState.IDLE);
            })
        .until(end);
  }

  // Default command?
  public static Command autoShoot(
      Shooter shooter, Drive drive, BooleanSupplier toggleOn) { // TODO unwind
    BooleanSupplier needsToUnwind = () -> shooter.needsToUnwind();
    BooleanSupplier end =
        () ->
            !AreaManager.isShootingArea(drive.getPose().getTranslation())
                || toggleOn.getAsBoolean()
                || needsToUnwind.getAsBoolean();

    return shoot(shooter, end); // BooleanSupplier toggleOn will be tied to toggle 1 being turned on
  }
}
