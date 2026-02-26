package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.wpilibj.DriverStation;

public class TurretIOSim implements TurretIO {
  private double requestedVoltage = 0;
  private double requestedAngle = 0;

  private double voltage = 0;
  private double currentAngle = 0;
  private double undefinedVoltage = -20;
  private double undefinedAngle = -1;

  private double slowRate = 0.02;
  private double fastRate = 1;
  private double rate;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.motorConnected = true;

    double prevPos = currentAngle;
    simPos();
    simVolts();
    double velocity = (currentAngle - prevPos) * 50;

    inputs.turretDegs = currentAngle;
    inputs.statorVolts = voltage;
    inputs.speedMotorRotations = velocity;
  }

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

  @Override
  public void setAngle(double angleDeg) {
    this.requestedAngle = angleDeg;
  }
}
