package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetection;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoIntake extends Command {
  // TODO adapt this to extend drive to pose

  private Drive drive;
  private VisionObjectDetection visionObjectDetection;
  private Supplier<Pose2d> currentPoseRequest = () -> new Pose2d();
  private Pose2d driveToPoseTarget;
  private Rotation2d targetAngle;
  private Translation2d fuelPose;
  private boolean autodrive;

  public AutoIntake(Drive drive, VisionObjectDetection visionObjectDetection, boolean autodrive) {
    this.drive = drive;
    this.visionObjectDetection = visionObjectDetection;
    this.autodrive = autodrive;
  }

  @Override
  public void initialize() {
    fuelPose = null;
    targetAngle = null;
  }

  @Override
  public void execute() {
    if (fuelPose == null) {
      fuelPose = visionObjectDetection.getBestFuelPose(true);
      if (fuelPose != null) {
        targetAngle =
            fuelPose.minus(drive.getPose().getTranslation()).getAngle().plus(Rotation2d.k180deg);
      }
    }

    if (fuelPose != null) {
      Logger.recordOutput("AutoIntake/fuelPosition", new Pose2d(fuelPose, Rotation2d.kZero));
      Logger.recordOutput(
          "AutoIntake/fuelAngleBotRelativeDeg",
          fuelPose.minus(drive.getPose().getTranslation()).getAngle().getDegrees());
      Logger.recordOutput("AutoIntake/driveHeadingDeg", drive.getPose().getRotation().getDegrees());
      Logger.recordOutput("AutoIntake/targetAngleDeg", targetAngle.getDegrees());
      if (autodrive) {
        driveToPoseTarget =
            new Pose2d(fuelPose, targetAngle)
                .transformBy(
                    new Transform2d(
                        new Translation2d(Constants.VisionObjectDetection.fuelIntakeOffset, 0),
                        new Rotation2d()));
        // TODO update drive to pose
      } else {
        // TODO drive.requestAutoRotate(targetAngle);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    // TODO end drive to pose
    // TODO drive.requestFieldRelativeMode();
  }
}
