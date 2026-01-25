package frc.robot.subsystems.shooter.tunnel;

import org.littletonrobotics.junction.AutoLog;

public interface TunnelIO {

  @AutoLog
  public static class TunnelIOInputs {
    public boolean motorConnected = false;
    public double voltage = 0.0;
    public double velocityRotationsPerSec = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double motorTempC = 0.0;
  }

  public default void updateInputs(TunnelIOInputs inputs) {}

  public default void setTargetVelocity(double velocity) {}

  public default void stop() {}

  public default void enableBrakeMode(boolean enable) {}

}
