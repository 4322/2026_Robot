package frc.robot.autonomous.modes.secondShallow;

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
import frc.robot.commands.UtilityCommands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LoggedTunableNumber;

public class LSecondShallowTrench extends SequentialCommandGroup {
  private PathPlannerPath firstPath;

  public LSecondShallowTrench(
      Drive drive, Intake intake, Shooter shooter, LoggedTunableNumber autoStartDelay) {
    firstPath = Robot.L_SECOND_SHALLOW_TRENCH;
    Pose2d startPoseBlue = firstPath.getStartingHolonomicPose().get();
    Pose2d startPoseRed = firstPath.flipPath().getStartingHolonomicPose().get();

    setName("L_SECOND_SHALLOW_TRENCH");
    addCommands(
        new InstantCommand(
            () -> {
              if (Robot.alliance == Alliance.Blue) {
                drive.setPose(startPoseBlue);
              } else {
                drive.setPose(startPoseRed);
              }
            }),
        new ParallelDeadlineGroup(
            new UtilityCommands.WaitSupplierCommand(autoStartDelay),
            ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake)),
        new ParallelCommandGroup(
            IntakeCommands.intake(intake), new WaitUntilCommand(() -> shooter.isHoodLowered())),
        new ParallelDeadlineGroup(
            AutoBuilder.followPath(firstPath),
            ShooterCommands.idle(shooter, intake, 0.0, 40.0, null),
            ShooterCommands.autoUnjam(shooter, Constants.Autonomous.unjamTimeSec)),
        ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake));
  }
}
