package frc.robot.subsystems.shooter.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

  @AutoLog
  public static class SpindexerIOInputs {
    public boolean motorConnected = false;
    public double voltage = 0.0;
    public double velocityRotationsPerSec = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double motorTempC = 0.0;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void setTargetVelocity(double velocity) {}

  public default void stop() {}

  public default void enableBrakeMode(boolean enable) {}

}
