package frc.robot.commands.Shooter;

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
        AreaManager areaManager = new AreaManager();
        BooleanSupplier needsToUnwind = shooter.needsToUnwind();
        BooleanSupplier inNonShootingArea = !areaManager.isShootingArea(drive.getPose());
        return Commands.run(() -> {
            shooter.setState(ShooterState.IDLE);
        }).until(needsToUnwind.getAsBoolean() || !inNonShootingArea.getAsBoolean());
    }
}
