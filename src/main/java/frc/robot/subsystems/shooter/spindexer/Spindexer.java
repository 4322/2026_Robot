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

  private SpindexerStates state = SpindexerStates.DISABLED;

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  public void inputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
  }

  public void outputsPeriodic() {
    switch (Constants.spindexerMode) {
      case TUNING -> {}
      case NORMAL -> {
        switch (state) {
          case DISABLED -> {
            if (DriverStation.isEnabled()) {
              state = SpindexerStates.IDLE;
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

    Logger.recordOutput("Spindexer/State", state.toString());
    Logger.recordOutput("Spindexer/RequestedSpeed", requestedSpeed);
  }

  public void requestIdle() {
    if (!unjamOverride) {
      state = SpindexerStates.IDLE;
      requestedSpeed = 0;
    }
  }

  public void requestGoal(double speed) {
    if (!unjamOverride) {
      state = SpindexerStates.INDEXING;
      requestedSpeed = speed;
    }
  }

  public void unjamOverride(boolean unjamOverride) {
    this.unjamOverride = unjamOverride;
    if (unjamOverride) {
      state = SpindexerStates.UNJAM;
      requestedSpeed = Constants.Spindexer.unjamRPS;
    } else {
      // Default to idle when unjam isn't desired
      state = SpindexerStates.IDLE;
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
