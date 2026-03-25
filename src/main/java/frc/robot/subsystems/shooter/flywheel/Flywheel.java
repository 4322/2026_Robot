package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class Flywheel {
  private FlywheelIO io;
  private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private double ballsShot = 0;
  private Timer hardwareTimer = new Timer();

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
    if (!(Math.abs(inputs.mechanismRPS - inputs.requestedMechanismRPS)
        < Constants.Flywheel.mechanismToleranceRPS)) {
      hardwareTimer.start();
    } else {
      hardwareTimer.stop();
      hardwareTimer.reset();
    }

    if (hardwareTimer.hasElapsed(
        Shooter.isScoring()
            ? Constants.scoringHardwareCheckTime
            : Constants.passingHardwareCheckTime)) {
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
