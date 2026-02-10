package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {
    public boolean motorConnected = false;
    public boolean followerMotorConnected = false;
    public double requestedMechanismRotations = 0.0;
    public double followerRequestedMechanismRotations = 0.0;
    public double actualMechanismRotations = 0.0;
   
    public double speedMotorRotations = 0.0;
    public double followerspeedMotorRotations = 0.0;
    public double appliedVolts = 0.0;
    public double followerAppliedVolts = 0.0;
    public double motorTempCelsius = 0.0;
    public double busCurrentAmps = 0.0;
     public double followerMotorTempCelsius = 0.0;
    public double followerBusCurrentAmps = 0.0;
    
    public boolean sensorConnected = false;
    public Color color = new Color(0, 0, 0);
    public double proximity = 0.0;

    public boolean fuelDetected = false;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void stop() {}

  public default TalonFX getTalonFX() {
    return null;
  }

  public default void enableBrakeMode(boolean enable) {}

  public default void setTargetMechanismRotations(double speedMechanismRotations) {}
}
