package frc.robot.subsystems.shooter.spindexer;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Spindexer {
  private SpindexerIO io;
  private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  private double requestedSpeed = -1;

  public enum SpindexerStates {
    DISABLED,
    IDLE,
    INDEXING
  }

  private SpindexerStates state = SpindexerStates.DISABLED;

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);

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
    }

    Logger.recordOutput("Spindexer/State", state.toString());
    Logger.recordOutput("Spindexer/RequestedSpeed", requestedSpeed);
  }

  public void requestIdle() {
    state = SpindexerStates.IDLE;
    requestedSpeed = 0;
  }

  public void requestIndex(double speed) {
    state = SpindexerStates.INDEXING;
    requestedSpeed = speed;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public boolean isStopped() {
    return inputs.mechanismRotationsPerSec
        < Constants.Spindexer.stoppedMechanismRotationsPerSec;
  }
}
