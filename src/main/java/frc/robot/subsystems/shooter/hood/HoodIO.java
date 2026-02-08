package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double rotations = 0.0;
    public double requestedAngle = 0.0;
    public double currentPulseWidth = 0.0;
    public double appliedVolts = 0.0;
    public double motorTempCelsius = 0.0;
    public double busCurrentAmps = 0.0;
    public boolean encoderConnected = false;
    public double encoderRotations = 0.0;
    public double encoderRotationsPerInfo = 0.0;
    public boolean servoEnabled = false;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

   public default void setEncoderPosition(double angle) {
  }

  public default void homingPulseWidth() {
  }

    public default void setServoPosition(double pulseWidth) {
    }

    public default void setServoVelocity(double pulseWidth) {
    }
}
