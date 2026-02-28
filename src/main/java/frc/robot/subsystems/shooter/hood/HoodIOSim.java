package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;

public class HoodIOSim implements HoodIO {
  private HoodIOInputs inputs = new HoodIOInputs();
  private double velocit = 0;
  private double rotations = 0;
  private int currentRequested = 1500;
  private double position = 0;
  private double pastDeg = 0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    simEstimatedPosition();
    inputs.encoderConnected = true;
    inputs.currentPulseWidth = currentRequested;
    inputs.rawRotations = rotations; // Convert degrees to rotations
    inputs.degrees = position; // Convert rotations to degrees
    inputs.encoderRPS = velocit;
    inputs.servoEnabled = true; // Assuming a threshold of 0.1A to determine if the servo is powered
  }

  @Override
  public void setEncoderHomed() {
    this.rotations = 0;
  }

  public void simEstimatedPosition() {

    position = ((Math.multiplyExact((290), 30) * velocit) + pastDeg);
    pastDeg = position;
  }

  @Override
  public void setServoVelocity(double velocity) {
    this.currentRequested = (1500 + ((int) MathUtil.clamp(velocity, -1, 1) * 500));
    velocit = ((int) MathUtil.clamp(velocity, -1, 1));
  }
}
