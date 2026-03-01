package frc.robot.subsystems.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.Constants;

public class GyroIOBoron implements GyroIO {
  private final Canandgyro gyro;

  public GyroIOBoron() {
    gyro = new Canandgyro(Constants.Drive.gyroID);

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
    inputs.connected = true;
    inputs.yawPosition = Rotation2d.fromRotations(gyro.getMultiturnYaw());
    inputs.yawVelocityRadPerSec = Units.rotationsToDegrees(gyro.getAngularVelocityYaw());
    inputs.odometryYawTimestamps = new double[100];
    inputs.odometryYawPositions = new Rotation2d[100];
    for (int i = 0; i <= 99; i++) {
      inputs.odometryYawTimestamps[i] = RobotController.getFPGATime() / 1e6;
      inputs.odometryYawPositions[i] = inputs.yawPosition;
    }
  }

  public Canandgyro getGyro() {
    return gyro;
  }
}
