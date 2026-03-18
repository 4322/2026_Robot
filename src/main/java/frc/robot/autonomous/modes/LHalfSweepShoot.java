package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetection;
import org.littletonrobotics.junction.Logger;

public class LHalfSweepShoot extends SequentialCommandGroup {

  public LHalfSweepShoot(Drive drive, LED led, Intake intake, Shooter shooter, Hood hood) {
    PathPlannerPath path = Robot.L_StartL_To_NeutralL_Intake;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
    Pose2d startPoseRed = path.flipPath().getStartingHolonomicPose().get();

    setName("L_HALF_SWEEP_SHOOT");
    if (Constants.turretLocked) {
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
              new SequentialCommandGroup(
                  AutoBuilder.followPath(Robot.L_StartL_To_NeutralL_Intake),
                  AutoBuilder.followPath(Robot.L_NeutralL_Intake_To_Mid),
                  AutoBuilder.followPath(Robot.L_NeutralL_Intake_Mid_Flip),
                  AutoBuilder.followPath(Robot.L_NeutralLMid_To_ShootL_LT) /* ,
                ShooterCommands.aimAndShoot(shooter, drive)*/)));
    } else {
      addCommands(
          new InstantCommand(
              () -> {
                Logger.recordOutput("Autonomous/AutoStarted", "RHalfSweepShoot");
                if (Robot.alliance == Alliance.Blue) {
                  drive.setPose(startPoseBlue);
                  Logger.recordOutput("Autonomous/StartPose", "startPoseBlue");
                } else {
                  drive.setPose(startPoseRed);
                  Logger.recordOutput("Autonomous/StartPose", "startPoseRed");
                }
              }),
          new ParallelCommandGroup(
              new SequentialCommandGroup(
                  ShooterCommands.trenchOverride(hood).withTimeout(1),
                  new WaitCommand(7),
                  ShooterCommands.trenchOverride(hood).withTimeout(2)),
              new ParallelCommandGroup(
                  IntakeCommands.setIntaking(intake),
                  new SequentialCommandGroup(
                      AutoBuilder.followPath(Robot.R_StartR_To_NeutralR_Intake),
                      AutoBuilder.followPath(Robot.R_NeutralR_Intake_To_Mid),
                      AutoBuilder.followPath(Robot.R_NeutralR_Intake_Mid_Flip),
                      AutoBuilder.followPath(Robot.R_NeutralRMid_To_ShootR)))));
    }
  }

  public LHalfSweepShoot(
      Drive drive, LED led, Intake intake, VisionObjectDetection visionObjectDetection) {
    PathPlannerPath path = Robot.R_StartR_To_NeutralR_Intake;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();

    Pose2d startPoseRed = path.flipPath().getStartingHolonomicPose().get();

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
        new SequentialCommandGroup(
            AutoBuilder.followPath(Robot.R_StartR_To_NeutralR_Intake),
            AutoBuilder.followPath(Robot.R_NeutralR_Intake_To_Mid),
            AutoBuilder.followPath(Robot.R_NeutralR_Intake_Mid_Flip),
            new ParallelCommandGroup(
                new AutoIntake(drive, visionObjectDetection, led, intake, false),
                AutoBuilder.followPath(Robot.R_NeutralRMid_To_ShootR))));
  }
}
