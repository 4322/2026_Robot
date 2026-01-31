package frc.robot.subsystems.vision.visionObjectDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;

public class VisionObjectDetection extends SubsystemBase {
  private final VisionObjectDetectionIOInputsAutoLogged inputs =
      new VisionObjectDetectionIOInputsAutoLogged();
  private final VisionObjectDetectionIO io;
  private final String hostname;
  private final Transform3d robotCenterToCamera;
  private final Drive drive;
  private Translation2d optimalFuelPosition;

  public VisionObjectDetection(String hostname, Drive drive, VisionObjectDetectionIO io) {
    this.hostname = hostname;
    this.drive = drive;
    this.io = io;
    this.robotCenterToCamera = Constants.VisionObjectDetection.robotCenterToCamera;
  }

  public enum ObjectDetectionType {
    COLOR,
    OBJECT
  }

  public Translation2d getTranslationToNearestFuel(Pose2d robotPose, boolean sameZone) {
    return new Translation2d();
  }
}
