package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;

public class TestLeave extends SequentialCommandGroup {
  public TestLeave(Drive drive) {
    setName("TEST_LEAVE");
    addCommands(
        new InstantCommand(
            () -> {
              PathPlannerPath path = Robot.TestLeave;
              if (Robot.alliance == Alliance.Red) {
                path = path.flipPath();
              }
              drive.setPose(path.getStartingHolonomicPose().get());
            }),
        AutoBuilder.followPath(Robot.TestLeave));
  }
}
