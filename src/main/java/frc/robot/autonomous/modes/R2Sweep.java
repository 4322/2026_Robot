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

public class R2Sweep extends SequentialCommandGroup {
  public R2Sweep(Drive drive, LED led, Intake intake, Shooter shooter) {
    PathPlannerPath path = Robot.R_StartR_To_NeutralR_Intake;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
    Pose2d startPoseRed = path.flipPath().getStartingHolonomicPose().get();

    setName("R_2_SWEEP");

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
                AutoBuilder.followPath(Robot.R_2SWEEP_A),
                AutoBuilder.followPath(Robot.R_2SWEEP_B),
                AutoBuilder.followPath(Robot.R_2SWEEP_CG),
                AutoBuilder.followPath(Robot.R_2SWEEP_D),
                AutoBuilder.followPath(Robot.R_2SWEEP_E),
                AutoBuilder.followPath(Robot.R_2SWEEP_F),
                AutoBuilder.followPath(Robot.R_2SWEEP_CG),
                AutoBuilder.followPath(Robot.R_2SWEEP_H)

            )));
  }
}
