package frc.robot.subsystems.shooter.flywheel;

import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.Outake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;

public class Flywheel {
  private FlywheelIO io;
  private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private double ballsShot = 0;
  private Timer hardwareTimer = new Timer();

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    Logger.recordOutput("Flywheel/BallsShot", ballsShot);
  }

  public void requestGoal(double velocity) {
    switch (Constants.flywheelMode) {
      case TUNING -> {}
      case NORMAL -> {
        io.setTargetMechanismRPS(velocity);
        inputs.requestedMechanismRPS = velocity;
      }
      case DISABLED -> {}
    }
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public boolean atTargetVelocity() {
  if (!(Math.abs(inputs.mechanismRPS - inputs.requestedMechanismRPS)
        < Constants.Flywheel.mechanismToleranceRPS)){
          hardwareTimer.start();
        }
  if (hardwareTimer.hasElapsed(Outake.isScoring() ?Constants.Flywheel.scoringHardwareCheckTime: Constants.Flywheel.passingHardwareCheckTime)){
        hardwareTimer.stop();
        hardwareTimer.reset();
        return true;
      } else {
    return Math.abs(inputs.mechanismRPS - inputs.requestedMechanismRPS)
        < Constants.Flywheel.mechanismToleranceRPS;
      }
  }

  public double getVelocity() {
    return inputs.mechanismRPS;
  }
}
