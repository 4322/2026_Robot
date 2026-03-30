package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class CDepotOutpost extends SequentialCommandGroup {
  public CDepotOutpost(Drive drive, LED led, Intake intake, Shooter shooter) {
    PathPlannerPath path = Robot.C_Start_To_Depot;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
    Pose2d startPoseRed = path.flipPath().getStartingHolonomicPose().get();
    setName("C_DEPOT_OUTPOST");

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
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                IntakeCommands.intake(intake),
                new WaitUntilCommand(() -> intake.hasExtended()),
                ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake)),
            new SequentialCommandGroup(
                AutoBuilder.followPath(Robot.C_Start_To_Depot),
                AutoBuilder.followPath(Robot.C_Depot_To_Outpost),
                new WaitCommand(Constants.Autonomous.smooshDelaySinglePass + 3),
                IntakeCommands.autoSmoosh(intake))));
  }
}
