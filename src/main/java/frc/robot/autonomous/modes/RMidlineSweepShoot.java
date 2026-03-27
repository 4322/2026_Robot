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
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;

public class RMidlineSweepShoot extends SequentialCommandGroup {

  public RMidlineSweepShoot(Drive drive, LED led, Intake intake, Shooter shooter) {
    PathPlannerPath path = Robot.R_StartR_To_NeutralR_Intake_Midline;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
    Pose2d startPoseRed = path.flipPath().getStartingHolonomicPose().get();

    setName("R_MIDLINE_SWEEP_SHOOT");
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
            IntakeCommands.intake(intake),
            new SequentialCommandGroup(
                AutoBuilder.followPath(Robot.R_StartR_To_NeutralR_Intake_Midline),
                AutoBuilder.followPath(Robot.R_NeutralR_Intake_Full_Midline),
                AutoBuilder.followPath(Robot.R_NeutralR_Intake_Full_Midline_Flip),
                AutoBuilder.followPath(Robot.R_NeutralRMid_To_ShootR),
                ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake),
                new WaitCommand(Constants.Autonomous.smooshDelaySinglePass),
                IntakeCommands.autoSmoosh(intake))));
  }
}
