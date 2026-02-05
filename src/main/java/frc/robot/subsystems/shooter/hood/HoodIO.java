package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double angleDeg = 0.0;
    public double requestedAngle = 0.0;
    public boolean motorConnected = false;
    public double appliedVolts = 0.0;
    public double motorTempCelsius = 0.0;
    public double busCurrentAmps = 0.0;
    public boolean encoderConnected = false;
  }

  public default void updateInputs(HoodIOInputs inputs) {}
}
