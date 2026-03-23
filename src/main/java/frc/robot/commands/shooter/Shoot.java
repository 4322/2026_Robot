package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends Command {
    public Shoot(Shooter shooter) {
      addRequirements(shooter);
    }
    
    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {

    }
    
    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished() {
        // In non shooting zone or manually inhibited
      return false;
    }
}
