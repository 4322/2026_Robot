package frc.robot.commands.Shooter;

import java.util.concurrent.BlockingDeque;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.shooter.areaManager.AreaManager;

public class ShooterCommand {


    public static Command turretUnwind(Shooter shooter) {
        BooleanSupplier rewindComplete = () -> shooter.isRewindComplete();
        return Commands.run(() -> shooter.setState(ShooterState.UNWIND), shooter)
        .until(rewindComplete);
    }

    public static Command shoot(Shooter shooter) {
        BooleanSupplier mechanismsAtSpeed = () -> shooter.isMechanismsAtSpeed();
        BooleanSupplier hoodAtAngle = () -> shooter.isHoodAtAngle();
        BooleanSupplier flywheelAtSpeed = () -> shooter.isFlywheelAtSpeed();

        return Commands.run(() -> {
            shooter.setState(ShooterState.PRESHOOT);
            if (flywheelAtSpeed.getAsBoolean()) {
                shooter.setState(ShooterState.SHOOT);
            }
        }, shooter);
    }

    public static Command idle(Shooter shooter) {
        return new InstantCommand(() -> shooter.setState(ShooterState.IDLE), shooter);
    }


    // Main commands

    public static Command inhibitAutoShoot(Shooter shooter) {
        return Commands.run(() -> {
            shooter.setState(ShooterState.IDLE);
        }, shooter);
    }

    public static Command areaInhibitAutoShoot(Shooter shooter, Drive drive) {
        BooleanSupplier needsToUnwind = () -> shooter.needsToUnwind();
        BooleanSupplier inShootingArea = () -> AreaManager.isShootingArea(drive.getPose().getTranslation());
        BooleanSupplier end = () -> needsToUnwind.getAsBoolean() || inShootingArea.getAsBoolean();
        return Commands.run(() -> {
            shooter.setState(ShooterState.IDLE);
        }).until(end);
    }

    public static Command autoShoot(Shooter shooter, Drive drive, BooleanSupplier toggleOn) {
        BooleanSupplier needsToUnwind = () -> shooter.needsToUnwind();
        BooleanSupplier inShootingArea = () -> AreaManager.isShootingArea(drive.getPose().getTranslation());
        BooleanSupplier end = () -> !AreaManager.isShootingArea(drive.getPose().getTranslation()) || toggleOn.getAsBoolean() || needsToUnwind.getAsBoolean();

        return Commands.run(() -> {
            switch(AreaManager.getZoneOfPosition(drive.getPose().getTranslation())) {
                case ALLIANCE_ZONE -> {

                }
                case LEFT_OPPOSITION -> {

                }
                case RIGHT_OPPOSITION -> {

                }
                case LEFT_NEUTRAL -> {

                }
                case RIGHT_NEUTRAL -> {

                }
            }
        }, shooter).until(end);
    }

}
