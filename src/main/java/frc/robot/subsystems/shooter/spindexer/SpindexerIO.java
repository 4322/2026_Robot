package frc.robot.subsystems.shooter.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

  @AutoLog
  public static class SpindexerIOInputs {
    public boolean leaderMotorConnected = false;
    public double leaderVoltage = 0.0;
    public double leaderMechanismRPS = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double leaderStatorCurrentAmps = 0.0;
    public double leaderMotorTempC = 0.0;

    public boolean followerMotorConnected = false;
    public double followerVoltage = 0.0;
    public double followerMechanismRPS = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerStatorCurrentAmps = 0.0;
    public double followerMotorTempC = 0.0;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void setTargetMechanismRotations(double velocity) {}

  public default void stop() {}

  public default void enableBrakeMode(boolean enable) {}
}
