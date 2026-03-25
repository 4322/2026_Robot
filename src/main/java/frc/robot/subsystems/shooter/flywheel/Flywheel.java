package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Flywheel {
  private FlywheelIO io;
  private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private double requestedSetpoint = 0;
  private double timerSetpoint = 0;
  private Timer setpointFallbackTimer = new Timer();
  private boolean fallbackToleranceEnabled = false;
  private Boolean isScoring = null;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public void inputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  public void outputsPeriodic() {
    switch (Constants.flywheelMode) {
      case TUNING -> {}
      case NORMAL -> {
        if (Constants.doubleToleranceEnabled) {
          // Check for change in setpoint to reset running timer
          if (timerSetpoint != requestedSetpoint) {
            timerSetpoint = requestedSetpoint;
            setpointFallbackTimer.stop();
            setpointFallbackTimer.reset();
          }
          // If in regular tolerance, reset timer
          else if (setpointFallbackTimer.isRunning()
              && MathUtil.isNear(
                  inputs.mechanismRPS,
                  requestedSetpoint,
                  Constants.Flywheel.mechanismToleranceRPS)) {
            setpointFallbackTimer.stop();
            setpointFallbackTimer.reset();
          }
          // Start timer upon entering larger tolerance
          else if (MathUtil.isNear(
              inputs.mechanismRPS,
              requestedSetpoint,
              Constants.Flywheel.mechanismFallbackToleranceRPS)) {
            setpointFallbackTimer.start();
          }

          // Different time thresholds based on scoring vs passing
          if (isScoring != null) {
            if (isScoring.booleanValue()
                && setpointFallbackTimer.hasElapsed(Constants.scoringDoubleToleranceTime)) {
              fallbackToleranceEnabled = true;
            } else if (!isScoring.booleanValue()
                && setpointFallbackTimer.hasElapsed(Constants.passingDoubleToleranceTime)) {
              fallbackToleranceEnabled = true;
            } else {
              fallbackToleranceEnabled = false;
            }
          } else {
            fallbackToleranceEnabled = false;
          }
        }
      }
      case DISABLED -> {}
    }
    Logger.recordOutput("Flywheel/usingFallbackTolerance", fallbackToleranceEnabled);
  }

  public void requestGoal(double velocity, Boolean isScoring) {
    switch (Constants.flywheelMode) {
      case TUNING -> {}
      case NORMAL -> {
        io.setTargetMechanismRPS(velocity);
        requestedSetpoint = velocity;
      }
      case DISABLED -> {}
    }
    this.isScoring = isScoring;
    Logger.recordOutput("Flywheel/RequestedSetpoint", requestedSetpoint);
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public boolean isAtGoal() {
    if (fallbackToleranceEnabled) {
      return Math.abs(inputs.mechanismRPS - requestedSetpoint)
          < Constants.Flywheel.mechanismFallbackToleranceRPS;
    } else {
      return Math.abs(inputs.mechanismRPS - requestedSetpoint)
          < Constants.Flywheel.mechanismToleranceRPS;
    }
  }
}
