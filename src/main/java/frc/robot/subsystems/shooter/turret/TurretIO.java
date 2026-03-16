package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean motorConnected = false;
    public double motorRPS = 0.0;
    public double appliedVolts = 0.0;
    public double statorVolts = 0.0;
    public double TempCelsius = 0.0;
    public int encoderOneCount = 0;
    public int encoderTwoCount = 0;
    public double turretDegs = 0.0;
    public double requestedTurretDegs = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setAngle(double angle) {}

  public default void setBrakeMode(boolean mode) {}

  public default void setPosition(double rot) {}
}
