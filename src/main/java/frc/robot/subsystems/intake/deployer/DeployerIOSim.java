package frc.robot.subsystems.intake.deployer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class DeployerIOSim implements DeployerIO {
  private double requestedVoltage = 0;
  private double requestedAngle = 0;

  private double voltage = 0;
  private double currentAngle =
      Constants.Deployer.maxGravityDegrees
          + Units.radiansToDegrees(Constants.Deployer.CANCoderStowed);
  private double undefinedVoltage = -20;

  private double slowRate = 0.02;
  private double fastRate = 1;

  @Override
  public void updateInputs(DeployerIOInputs inputs) {
    inputs.connected = true;

    double prevPos = currentAngle;
    simPos();
    simVolts();
    double velocity = (currentAngle - prevPos) * 50;

    inputs.angleDeg = currentAngle;
    inputs.appliedVolts = voltage;
    inputs.motorRotationsPerSec = velocity;
  }

  @Override
  public void setPosition(double requestedPosDeg) {
    this.requestedAngle = requestedPosDeg;
  }

  @Override
  public void enableBrakeMode(boolean mode) {}

  @Override
  public void stop() {}

  private void simVolts() {
    if (DriverStation.isEnabled()) {
      if (requestedVoltage == undefinedVoltage) {
        voltage = 0;
      } else if (voltage < requestedVoltage) {
        voltage += (requestedVoltage - voltage) * fastRate;
      } else {
        voltage -= (voltage - requestedVoltage) * fastRate;
      }
    }
  }

  private void simPos() {
    if (DriverStation.isEnabled()) {
      if (currentAngle < requestedAngle) {
        currentAngle += (requestedAngle - currentAngle) * fastRate;
      } else {
        currentAngle -= (currentAngle - requestedAngle) * fastRate;
      }
    }
  }
}
