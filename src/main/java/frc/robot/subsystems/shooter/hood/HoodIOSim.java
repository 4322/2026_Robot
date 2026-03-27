package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;

public class HoodIOSim implements HoodIO {
  private double velocity = 0;
  private double rotations = 0;
  private double position = 0;
  private double maxRPS = 0.2;
  private double degreePerSecond = 37;
  private double prevPosition = 0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.encoderConnected = true;
    inputs.encoderRotations = rotations; // Convert degrees to rotations
    inputs.degrees = position;
    inputs.encoderRPS = velocity;
    inputs.servoEnabled = true;
  }

  @Override
  public void setEncoderHomed() {
    this.rotations = 0;
    this.position = 0;
  }

  @Override
  public void simEstimatedPosition() {

    this.position =
        MathUtil.clamp((this.maxRPS * this.degreePerSecond * this.velocity), 0, 38) + prevPosition;
    prevPosition = position;
  }

  @Override
  public void setServoVelocity(double velocity, double requestedAngleDeg) {
    this.velocity = ((int) MathUtil.clamp(velocity, -1, 1));
    simEstimatedPosition();
  }
}
