package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean motorConnected = false;
    public double speedMotorRotations = 0.0;
    public double appliedVolts = 0.0;
    public double motorTempCelsius = 0.0;
    public double encoderOneRotations = 0.0;
    public double encoderTwoRotations = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setAngle(double angle) {}

  public default void setBreakMode(boolean mode) {}

  public default void rewind() {}
}
