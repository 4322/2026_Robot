package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
  @AutoLog
  public static class RollersIOInputs {
    public boolean connected = false;
    public double statorCurrentAmps = 0.0;
    public double busCurrentAmps = 0.0;
    public double motorTempCelcius = 0.0;
    public double motorRotationsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double controllerTempCelcius = 0.0;
  }

  public default void updateInputs(RollersIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void stopMotor() {}

  public default TalonFX getTalonFX() {
    return null;
  }

  public default void enableBreakMode(boolean mode) {}
}
