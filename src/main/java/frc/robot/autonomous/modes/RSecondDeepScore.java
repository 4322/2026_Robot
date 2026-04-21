package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousSelector.AutoStartPosition;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RSecondDeepScore extends SequentialCommandGroup {
  private PathPlannerPath firstPath;

  public RSecondDeepScore(
      Drive drive,
      Intake intake,
      Shooter shooter,
      LoggedDashboardChooser<AutoStartPosition> startPositionSelector,
      LoggedTunableNumber autoStartDelay) {
    if (startPositionSelector.get() == AutoStartPosition.INSIDE_TRENCH) {
      firstPath = Robot.R_SECONDDEEP_A;
    } else {
      firstPath = Robot.R_SECONDDEEP_A_OUT;
    }
    Pose2d startPoseBlue = firstPath.getStartingHolonomicPose().get();
    Pose2d startPoseRed = firstPath.flipPath().getStartingHolonomicPose().get();

    setName("R_SECONDDEEP_SCORE");
    addCommands(
        new InstantCommand(
            () -> {
              if (Robot.alliance == Alliance.Blue) {
                drive.setPose(startPoseBlue);
              } else {
                drive.setPose(startPoseRed);
              }
            }),
        new WaitCommand(autoStartDelay.get()),
        IntakeCommands.intake(intake),
        AutoBuilder.followPath(firstPath),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                AutoBuilder.followPath(Robot.R_SECONDDEEP_B_SCORE),
                AutoBuilder.followPath(Robot.R_SECONDDEEP_C)
            ),
            ShooterCommands.idle(shooter, intake, 15.0, 40.0, null),
            ShooterCommands.autoUnjam(shooter, Constants.Autonomous.unjamTimeSec)),
        new ParallelCommandGroup(
            ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake),
            IntakeCommands.autoSmoosh(intake, 2, 5)
        )
        );
  }
}
