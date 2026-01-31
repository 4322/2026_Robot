package frc.robot.subsystems.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class GyroIOBoron implements GyroIO {
    private final Canandgyro gyro;

  public GyroIOBoron() {
    gyro = new Canandgyro(0); //TODO set this

    CanandgyroSettings settings = new CanandgyroSettings();
    settings.setAngularVelocityFramePeriod(0.02);
    CanandgyroSettings gyroConfigStatus = gyro.setSettings(settings, 0.02, 5);

    if (!gyroConfigStatus.isEmpty()) {
      DriverStation.reportError("Gyro failed to configure", false);
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // check connection with 60ms timeout instead of 2s default due to frequency of gyro reporting
    // failure
  }

  public Canandgyro getGyro() {
    return gyro;
  }
}
