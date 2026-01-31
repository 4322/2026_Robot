package frc.robot.subsystems.shooter.flywheel;

import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOInputs;
import org.littletonrobotics.junction.Logger;

public class Flywheel {
  private FlywheelIO io;
  private FlywheelIOInputs inputs = new FlywheelIOInputs();
  private double ballsShot = 0;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    // Logger.recordOutput("Flywheel/Inputs", inputs);
    if (inputs.fuelDetectedOutputting) {
      ballsShot += 1;
    }
    Logger.recordOutput("Flywheel/BallsDetectedShot", ballsShot);
  }

  public void setTargetVelocity(double velocityRPS) {
    inputs.requestedSpeed = velocityRPS;
    io.setTargetVelocity(velocityRPS);
  }

  public void stop() {
    io.stop();
  }

  public void setIdleVelocity(double velocity) {
    inputs.requestedSpeed = velocity;
    io.setTargetVelocity(Constants.Flywheel.idleShootSpeedRPS);
  }

  public boolean atTargetVelocity() {
    return Math.abs(inputs.actualSpeed - inputs.requestedSpeed)
        < Constants.Flywheel.allowedVelocityErrorRPS;
  }
}
