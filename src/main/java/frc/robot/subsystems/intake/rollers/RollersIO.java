package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
  @AutoLog
  public static class RollersIOInputs {
    public boolean leaderConnected = false;
    public double leaderStatorAmps = 0.0;
    public double leaderBusAmps = 0.0;
    public double leaderMotorTempCelcius = 0.0;
    public double leaderRotationsPerSec = 0.0;
    public double leaderVolts = 0.0;
    public double leaderControllerTempCelcius = 0.0;

    public boolean followerConnected = false;
    public double followerStatorAmps = 0.0;
    public double followerBusAmps = 0.0;
    public double followerMotorTempCelcius = 0.0;
    public double followerRotationsPerSec = 0.0;
    public double followerVolts = 0.0;
    public double followerControllerTempCelcius = 0.0;
  }

  public default void updateInputs(RollersIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void stopMotor() {}

  public default TalonFX getTalonFX() {
    return null;
  }

  public default void enableBrakeMode(boolean enable) {}
}
