package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;

public class L2056 extends SequentialCommandGroup {
  public L2056(Drive drive, LED led, Intake intake, Shooter shooter) {
    PathPlannerPath path = Robot.L_2056_A;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
    Pose2d startPoseRed = path.flipPath().getStartingHolonomicPose().get();

    setName("L_2056");
    addCommands(
        new InstantCommand(
            () -> {
              if (Robot.alliance == Alliance.Blue) {
                drive.setPose(startPoseBlue);
              } else {
                drive.setPose(startPoseRed);
              }
            }),
        IntakeCommands.intake(intake),
        AutoBuilder.followPath(Robot.L_2056_A),
        new ParallelDeadlineGroup(
            AutoBuilder.followPath(Robot.L_2056_B),
            ShooterCommands.idle(shooter, intake, 15.0, 40.0, 172.0),
            ShooterCommands.autoUnjam(shooter, Constants.Autonomous.unjamTimeSec)),
        new ParallelDeadlineGroup(
            AutoBuilder.followPath(Robot.L_2056_C),
            ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake),
            IntakeCommands.autoSmoosh(
                intake,
                Constants.Autonomous.smooshDelayFirst2056,
                Constants.Autonomous.twoSweepShootTimeFirstPass)),
        IntakeCommands.intake(intake),
        new WaitUntilCommand(() -> shooter.isHoodLowered()),
        AutoBuilder.followPath(Robot.L_2056_D),
        new ParallelDeadlineGroup(
            AutoBuilder.followPath(Robot.L_2056_B),
            ShooterCommands.idle(shooter, intake, 15.0, 40.0, 172.0),
            ShooterCommands.autoUnjam(shooter, Constants.Autonomous.unjamTimeSec)),
        new ParallelDeadlineGroup(
            AutoBuilder.followPath(Robot.L_2056_C),
            ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake),
            IntakeCommands.autoSmoosh(
                intake,
                Constants.Autonomous.smooshDelaySecond2056,
                Constants.Autonomous.twoSweepShootTimeFirstPass)),
        IntakeCommands.intake(intake),
        AutoBuilder.followPath(Robot.L_2056_G));
  }
}
