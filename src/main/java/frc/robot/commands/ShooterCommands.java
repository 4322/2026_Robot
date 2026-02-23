package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {

  public static Command shoot(Shooter shooter) {

    return Commands.run(() -> shooter.requestShoot(), shooter);
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

    return shoot(shooter);
  }
}
