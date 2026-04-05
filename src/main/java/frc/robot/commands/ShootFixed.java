package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.fixedAreaPlacement;

public class ShootFixed extends Command {
  private Shooter shooter;
  private fixedAreaPlacement fixedAreaPlacement;

  public ShootFixed(Shooter shooter, fixedAreaPlacement fixedAreaPlacement) {
    this.shooter = shooter;
    this.fixedAreaPlacement = fixedAreaPlacement;
  }

  @Override
  public void execute() {
    shooter.requestShoot(true, true, fixedAreaPlacement);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.requestIdle(null, null);
  }
}
