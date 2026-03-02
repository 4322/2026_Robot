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
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetection;

public class RHalfSweepShoot extends SequentialCommandGroup {

  public RHalfSweepShoot(Drive drive, LED led, Intake intake) {
    PathPlannerPath path = Robot.R_StartR_To_NeutralR_Intake;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
    path.flipPath();
    Pose2d startPoseRed = path.getStartingHolonomicPose().get();

    setName("R_HALF_SWEEP_SHOOT");
    addCommands(
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
            AutoBuilder.followPath(Robot.R_StartR_To_NeutralR_Intake)
                .andThen(AutoBuilder.followPath(Robot.R_NeutralR_Intake_To_Mid))
                .andThen(AutoBuilder.followPath(Robot.R_NeutralR_Intake_Mid_Flip))
                .andThen(AutoBuilder.followPath(Robot.R_NeutralRMid_To_ShootR))));
  }

  public RHalfSweepShoot(
      Drive drive, LED led, Intake intake, VisionObjectDetection visionObjectDetection) {
    PathPlannerPath path = Robot.R_StartR_To_NeutralR_Intake;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
    path.flipPath();
    Pose2d startPoseRed = path.getStartingHolonomicPose().get();

    setName("R_HALF_SWEEP_SHOOT_OD");
    addCommands(
        new InstantCommand(
            () -> {
              if (Robot.alliance == Alliance.Blue) {
                drive.setPose(startPoseBlue);
              } else {
                drive.setPose(startPoseRed);
              }
            }),
        new ParallelCommandGroup(IntakeCommands.setIntaking(intake)),
        AutoBuilder.followPath(Robot.R_StartR_To_NeutralR_Intake)
            .andThen(AutoBuilder.followPath(Robot.R_NeutralR_Intake_To_Mid))
            .andThen(AutoBuilder.followPath(Robot.R_NeutralR_Intake_Mid_Flip))
            .andThen(
                new ParallelCommandGroup(
                    new AutoIntake(drive, visionObjectDetection, led, intake, false),
                    AutoBuilder.followPath(Robot.R_NeutralRMid_To_ShootR))));
  }
}
