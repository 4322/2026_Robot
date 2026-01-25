package frc.robot.subsystems.vision.visionObjectDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionObjectDetection extends SubsystemBase {
    public VisionObjectDetection() {
    }

    public enum ObjectDetectionType {
        COLOR,
        OBJECT
    }

    public Translation2d getTranslationToNearestFuel(Pose2d robotPose, boolean sameZone) {
        return new Translation2d();
    }

}
