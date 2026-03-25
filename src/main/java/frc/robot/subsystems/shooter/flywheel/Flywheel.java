package frc.robot.subsystems.shooter.flywheel;

import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Flywheel {
  private FlywheelIO io;
  private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private double ballsShot = 0;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public void inputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  public void outputsPeriodic() {}

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
    return Math.abs(inputs.mechanismRPS - inputs.requestedMechanismRPS)
        < Constants.Flywheel.mechanismToleranceRPS;
  }

  public double getVelocity() {
    return inputs.mechanismRPS;
  }
}
