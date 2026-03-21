package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetection;
import org.littletonrobotics.junction.Logger;

public class RFullSweepShoot extends SequentialCommandGroup {
  public RFullSweepShoot(Drive drive, LED led, Intake intake, Shooter shooter) {
    PathPlannerPath path = Robot.R_StartR_To_NeutralR_Intake;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
    Pose2d startPoseRed = path.flipPath().getStartingHolonomicPose().get();
    setName("R_FULL_SWEEP_SHOOT");
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
            IntakeCommands.setIntaking(intake),
            new SequentialCommandGroup(
                AutoBuilder.followPath(Robot.R_StartR_To_NeutralR_Intake),
                AutoBuilder.followPath(Robot.R_NeutralR_Intake_Full),
                AutoBuilder.followPath(Robot.R_NeutralR_Intake_Full_Flip),
                AutoBuilder.followPath(Robot.R_Neutral_Mid_To_ShootR))));
  }

  public RFullSweepShoot(
      Drive drive, LED led, Intake intake, VisionObjectDetection visionObjectDetection) {

    PathPlannerPath path = Robot.R_StartR_To_NeutralR_Intake;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
    Pose2d startPoseRed = path.flipPath().getStartingHolonomicPose().get();

    setName("R_FULL_SWEEP_SHOOT_OD");
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
        new ParallelCommandGroup(IntakeCommands.setIntaking(intake)),
        new SequentialCommandGroup(
            AutoBuilder.followPath(Robot.R_StartR_To_NeutralR_Intake),
            AutoBuilder.followPath(Robot.R_NeutralR_Intake_Full),
            AutoBuilder.followPath(Robot.R_NeutralR_Intake_Full_Flip),
            new ParallelCommandGroup(
                new AutoIntake(drive, visionObjectDetection, led, intake, false),
                AutoBuilder.followPath(Robot.R_Neutral_Mid_To_ShootR))));
  }
}
