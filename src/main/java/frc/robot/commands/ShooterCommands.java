package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;

public class ShooterCommands {

  public static Command shoot(Shooter shooter, BooleanSupplier end) {

    return Commands.run(() -> shooter.requestShoot()).until(end);
  }

  // Main commands

  public static Command inhibitAutoShoot(Shooter shooter) {

    return Commands.run(
        () -> {
          shooter.requestIdle();
        },
        shooter);
  }

  // Default command
  public static Command autoShoot(Shooter shooter) {

    BooleanSupplier end = () -> false;

    return shoot(shooter, end);
  }
}
