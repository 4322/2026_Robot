package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

public class ShooterCommand {


    public static Command turretUnwind(Shooter shooter) {
        BooleanSupplier rewindNotComplete = () -> !shooter.isRewindComplete();
        return Commands.run(() -> shooter.setState(ShooterState.UNWIND), shooter)
        .onlyWhile(rewindNotComplete);
    }

    public static Command shoot(Shooter shooter) {
        BooleanSupplier mechanismsAtSpeed = () -> shooter.isMechanismsAtSpeed();
        BooleanSupplier hoodAtAngle = () -> shooter.isHoodAtAngle();

        return Commands.run(() -> shooter.setState(ShooterState.SHOOTING))
    }
}
