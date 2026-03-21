package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.hood.Hood;

public class LHalfSweepShoot extends SequentialCommandGroup {

  public LHalfSweepShoot(Drive drive, LED led, Intake intake, Shooter shooter, Hood hood) {
    PathPlannerPath path = Robot.L_StartL_To_NeutralL_Intake;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
    Pose2d startPoseRed = path.flipPath().getStartingHolonomicPose().get();

    setName("L_HALF_SWEEP_SHOOT");

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
                AutoBuilder.followPath(Robot.L_NeutralLMid_To_ShootL))));
  }
}
