package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class CenterStartToDepot extends SequentialCommandGroup {
  public CenterStartToDepot(Drive drive, Intake intake, Shooter shooter) {
    PathPlannerPath path = Robot.C_To_Depot;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
    Pose2d startPoseRed = path.flipPath().getStartingHolonomicPose().get();
    setName("C_Start_To_DEPOT");

    addCommands(
        new InstantCommand(() -> Logger.recordOutput("Autonomous/autoStarted", true)),
        new InstantCommand(
            () -> {
              if (Robot.alliance == Alliance.Blue) {
                drive.setPose(startPoseBlue);
              } else {
                drive.setPose(startPoseRed);
              }
            }),
        new SequentialCommandGroup(
            IntakeCommands.intake(intake),
            new WaitUntilCommand(() -> intake.hasExtended()),
            new ParallelCommandGroup(ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake))
                .withTimeout(2),
            AutoBuilder.followPath(Robot.C_To_Depot),
            new ParallelDeadlineGroup(
                ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake),
                IntakeCommands.autoSmoosh(
                    intake,
                    Constants.Autonomous.twoSweepSmooshDelayFirstPass,
                    Constants.Autonomous.twoSweepShootTimeFirstPass))));
  }
}
