package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;

public interface FlywheelIO {

@AutoLog
public static class FlywheelIOInputs {
public double requestedSpeed = 0.0;
public double rotationsPerSecond = 0.0;
public double appliedVolts = 0.0;
public double motorTempCelsius = 0.0;
public double sensorColorYellow = 0.0;

public boolean fuelDetectedOutputting = false;
public double busCurrentAmps = 0.0;
    
}

public default void updateInputs(FlywheelIOInputs inputs) {
}

public default void setVoltage(double speedRPS) {
}

public default void stop() {
}

public default TalonFX getTalonFX() {
    return null;
}
}