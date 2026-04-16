package frc.robot.subsystems.shooter.hood;

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
    inputs.hoodDegrees = position;
  }

  @Override
  public void setEncoderHomed() {
    this.rotations = 0;
    this.position = 0;
  }
}
