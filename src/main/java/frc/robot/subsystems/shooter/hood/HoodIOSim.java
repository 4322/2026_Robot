package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;

public class HoodIOSim implements HoodIO {
  private double velocity = 0;
  private double rotations = 0;
  private int currentRequested = 1500;
  private double position = 0;
  private double maxRPM = 0.2; 
  private double degreePerSecond = 37;

  @Override
  public void updateInputs(HoodIOInputs inputs) {

    simEstimatedPosition();
    setServoVelocity(velocity);
    inputs.encoderConnected = true;
    inputs.currentPulseWidth = currentRequested;
    inputs.encoderRotations = rotations; // Convert degrees to rotations
    inputs.degrees = position; // Convert rotations to degrees
    inputs.encoderRPS = velocity;
    inputs.servoEnabled = true; // Assuming a threshold of 0.1A to determine if the servo is powered
  }

  @Override
  public void setEncoderHomed() {
    this.rotations = 0;
    this.position = 0;
  }

  @Override
  public void simEstimatedPosition() {
    this.position = MathUtil.clamp((this.maxRPM * this.degreePerSecond * this.velocity), 0, 37);
  }

  @Override
  public void setServoVelocity(double velocity) {
    this.currentRequested = (1500 + ((int) MathUtil.clamp(velocity, -1, 1) * 500));
    this.velocity = ((int) MathUtil.clamp(velocity, -1, 1));
  }
}
