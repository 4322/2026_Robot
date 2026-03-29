package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double encoderRotations = 0.0;
    public double servoAmps = 0.0;
    public boolean encoderConnected = false;
    public double encoderRPS = 0.0;
    public boolean servoEnabled = false;
    public double hoodDegrees = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setEncoderHomed() {}

  public default void setPulseWidth(int pulseWidth) {}

  public default void simEstimatedPosition() {}
}
