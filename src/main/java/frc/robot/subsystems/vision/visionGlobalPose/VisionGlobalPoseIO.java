package frc.robot.subsystems.vision.visionGlobalPose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionGlobalPoseIO {
  @AutoLog
  public static class VisionGlobalPoseIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public GlobalPoseObservation[] globalPoseObservations = new GlobalPoseObservation[0];
    public int singleTagFiducialID = 1;
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record GlobalPoseObservation(
      double timestamp,
      Pose3d pose,
      Pose3d altPose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      double averageTagDistanceAlt,
      boolean useMultiTag) {}

  public default void updateInputs(VisionGlobalPoseIOInputs inputs) {}
}
