package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {
    public boolean motorConnected = false;
    public double requestedMechanismRotations = 0.0;
    public double actualMechanismRotations = 0.0;
    public double speedMotorRotations = 0.0;
    public double appliedVolts = 0.0;
    public double motorTempCelsius = 0.0;
    public double colorYellow = 0.0;
    public double distance = 0.0;

    public boolean fuelDetectedOutputting = false;
    public double busCurrentAmps = 0.0;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setVoltage(double speedMechanismRotations) {}

  public default void stop() {}

  public default TalonFX getTalonFX() {
    return null;
  }

  public default void enableBrakeMode(boolean enable) {}

  public default void setTargetMechanismRotations(double velocity) {}
}
