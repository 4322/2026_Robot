package frc.robot.subsystems.vision.visionGlobalPose;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;

public class VisionGlobalPose {
  public VisionGlobalPose() {}

  private Pose2d getVisionGlobalPose() {
    return new Pose2d();
  }

  // Returns hybrid pose of vision and odometry
  public Pose2d getHybridPose(Drive drive) {
    return new Pose2d();
  }
}
