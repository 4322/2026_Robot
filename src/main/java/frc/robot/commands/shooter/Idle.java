package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.BallPath;
import frc.robot.subsystems.shooter.Outake;

public class Idle extends Command {
  private Outake outake;
  private BallPath ballPath;

  public Idle(Outake outake, BallPath ballPath) {
    this.outake = outake;
    this.ballPath = ballPath;
    addRequirements(outake);
  }

  @Override
  public void execute() {
    ballPath.setBallPathIdle();
    outake.setOutakeIdle();
  }
}
