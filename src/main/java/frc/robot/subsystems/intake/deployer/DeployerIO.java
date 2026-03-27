package frc.robot.subsystems.intake.deployer;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface DeployerIO {

  @AutoLog
  public static class DeployerIOInputs {
    public boolean connected = false;
    public double statorCurrentAmps = 0.0;
    public double busCurrentAmps = 0.0;
    public double motorTempCelcius = 0.0;
    public double motorDegreesPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double angleDeg = 0.0;
    public double encoderRotations = 0.0;
    public double motorRotations;
  }

  public default void updateInputs(DeployerIOInputs inputs) {}

  public default void setPosition(double requestedPosDeg) {}

  public default void setBrakeMode(boolean mode) {}

  public default void stop() {}

  public default TalonFX getTalonFX() {
    return null;
  }

  public default void seedPosition(double newAngle) {}
}
