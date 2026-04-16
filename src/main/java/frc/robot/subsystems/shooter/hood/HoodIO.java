package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean motorConnected = false;
    public boolean encoderConnected = false;
    public double motorRPS = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double tempCelsius = 0.0;
    public double encoderRot = 0.0;
    public double hoodDegrees = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setEncoderHomed() {}

  public default void setPulseWidth(int pulseWidth) {}

  public default void simEstimatedPosition() {}
}
