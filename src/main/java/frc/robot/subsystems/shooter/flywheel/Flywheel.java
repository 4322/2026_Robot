package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Flywheel {
  private FlywheelIO io;
  private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private double requestedMechanismRPS = 0.0;
  private double ballsShot = 0;
  private boolean fuelDetected = false;

  public enum FlywheelStates {
    DISABLED,
    ENABLED,
    SHOOTING
  }

  private FlywheelStates state = FlywheelStates.DISABLED;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    Logger.recordOutput("Flywheel/BallsShot", ballsShot)
    

    if (inputs.fuelDetected && !fuelDetected) {
      ballsShot++;
      fuelDetected = true;
    } else if (!inputs.fuelDetected) {
      fuelDetected = false;
    }

    
      
    

    Logger.recordOutput("Flywheel/State", state.toString());
  }

 

  public void requestGoal(double velocity) {
    io.setTargetMechanismRotations(velocity);
    inputs.requestedMechanismRotations = velocity;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public boolean isFuelDetected() {
    return inputs.fuelDetected;
  }

  public boolean atTargetVelocity() {
    return Math.abs(inputs.actualMechanismRotations - inputs.requestedMechanismRotations)
        < Constants.Flywheel.allowedVelocityErrorMechanismRPS;
  }

  public double getVelocity() {
    return inputs.actualMechanismRotations;
  }
}
