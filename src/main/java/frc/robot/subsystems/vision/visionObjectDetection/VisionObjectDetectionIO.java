package frc.robot.subsystems.vision.visionObjectDetection;

import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionObjectDetectionIO {

  public void updateInputs(VisionObjectDetectionIOInputsAutoLogged inputs);

  @AutoLog
  public static class VisionObjectDetectionIOInputs {
    public boolean hasTarget;
    public Rotation3d[] visibleObjectRotations;

    public double latestResultTimestamp;
  }
}
