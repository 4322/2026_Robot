package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.BallPath;
import frc.robot.subsystems.shooter.Outake;

public class Unjam extends Command {
  private Outake outake;
  private BallPath ballPath;

  public Unjam(Outake outake, BallPath ballPath) {
    this.outake = outake;
    this.ballPath = ballPath;
    addRequirements(outake);
  }

  @Override
  public void execute() {
    ballPath.setBallPathUnjam();
  }

  @Override
  public void end(boolean interrupted) {
    ballPath.setBallPathIdle();
    outake.setOutakeIdle();
  }

  @Override
  public boolean isFinished() {
    // In non shooting zone or manually inhibited
    return (!RobotContainer.controller.b().getAsBoolean());
  }
}
