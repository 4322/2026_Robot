package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean motorConnected = false;
    public double speedMotorRotations = 0.0;
    public double appliedVolts = 0.0;
    public double motorTempCelsius = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}
}
