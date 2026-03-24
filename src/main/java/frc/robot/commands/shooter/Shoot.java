package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.BallPath;
import frc.robot.subsystems.shooter.Outake;

public class Shoot extends Command {
  private Outake outake;
  private BallPath ballPath;

  public Shoot(Outake outake, BallPath ballPath) {
    this.outake = outake;
    this.ballPath = ballPath;
    addRequirements(outake);
  }

  @Override
  public void execute() {
    outake.setOutakeShoot();
    if (outake.isHoodAtPosition() && outake.isFlywheelAtSpeed() && outake.isTurretAtPosition() && !RobotContainer.controller.b().getAsBoolean()) {
      ballPath.setBallPathShoot();
    } else if (!RobotContainer.controller.b().getAsBoolean()){
      ballPath.setBallPathIdle();
    } else {
       ballPath.setBallPathUnjam();
    }
  }

  @Override
  public void end(boolean interrupted) {
      ballPath.setBallPathIdle();
      outake.setOutakeIdle();
  }

  @Override
  public boolean isFinished() {
    // In non shooting zone or manually inhibited
    return (!outake.isDriveInShootingArea() && DriverStation.isTeleopEnabled())
        || !RobotContainer.controller.rightTrigger().getAsBoolean()
        || outake.restrictAllianceShoot();
  }
}
