package frc.robot.subsystems.shooter.spindexer;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Spindexer {
  private SpindexerIO io;
  private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  private double requestedSpeed = -1;
  private boolean unjamOverride;

  public enum SpindexerStates {
    DISABLED,
    IDLE,
    INDEXING,
    UNJAM
  }

  private SpindexerStates statee = SpindexerStates.DISABLED;

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  public void inputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Spindexer", inputs);
  }

  public void outputsPeriodic() {
    switch (Constants.spindexerMode) {
      case TUNING -> {}
      case NORMAL -> {
        if (DriverStation.isDisabled()) {
          statee = SpindexerStates.DISABLED;
        }

        switch (statee) {
          case DISABLED -> {
            // Reset variables
            unjamOverride = false;
            if (DriverStation.isEnabled()) {
              statee = SpindexerStates.IDLE;
            }
          }
          case IDLE -> {
            io.stop();
          }
          case INDEXING -> {
            io.setTargetMechanismRotations(requestedSpeed);
          }
          case UNJAM -> {
            io.setTargetMechanismRotations(Constants.Spindexer.unjamRPS);
          }
        }
      }
      case DISABLED -> {}
    }

    Logger.recordOutput("Shooter/Spindexer/State", statee.toString());
    Logger.recordOutput("Shooter/Spindexer/RequestedSpeed", requestedSpeed);
    Logger.recordOutput("Shooter/Spindexer/Stopped", isStopped());
  }

  public void requestIdle() {
    if (!unjamOverride) {
      statee = SpindexerStates.IDLE;
      requestedSpeed = 0;
    }
  }

  public void requestGoal(double speed) {
    if (!unjamOverride) {
      statee = SpindexerStates.INDEXING;
      requestedSpeed = speed;
    }
  }

  public void unjamOverride(boolean unjamOverride) {
    this.unjamOverride = unjamOverride;
    if (unjamOverride) {
      statee = SpindexerStates.UNJAM;
      requestedSpeed = Constants.Spindexer.unjamRPS;
    } else {
      // Default to idle when unjam isn't desired
      statee = SpindexerStates.IDLE;
      requestedSpeed = 0;
    }
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public boolean isStopped() {
    return inputs.mechanismRPS < Constants.Spindexer.stoppedMechanismRotationsPerSec;
  }

  public double getVelocity() {
    return inputs.mechanismRPS;
  }
}
