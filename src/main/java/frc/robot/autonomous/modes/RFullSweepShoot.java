package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.AutoIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetection;

public class RFullSweepShoot extends SequentialCommandGroup {
  public RFullSweepShoot(Drive drive, VisionObjectDetection visionObjectDetection, LED led) {
    setName("R_FULL_SWEEP_SHOOT");
    addCommands(
        new InstantCommand(() -> {if (Robot.alliance == Alliance.Blue) {
                  drive.setPose(Robot.R_StartR_To_NeutralR_Intake.getStartingHolonomicPose().get());
                } else {
                  drive.setPose(Robot.R_StartR_To_NeutralR_Intake.flipPath().getStartingHolonomicPose().get());
                }}),
        AutoBuilder.followPath(Robot.R_StartR_To_NeutralR_Intake),
        AutoBuilder.followPath(Robot.R_NeutralR_Intake_Full),
        AutoBuilder.followPath(Robot.R_NeutralR_Intake_Full_Flip),
        new AutoIntake(drive, visionObjectDetection, led, true),
        AutoBuilder.followPath(Robot.R_Neutral_Mid_To_ShootR));
  }
}
