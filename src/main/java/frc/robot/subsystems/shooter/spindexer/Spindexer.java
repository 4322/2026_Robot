package frc.robot.subsystems.shooter.spindexer;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Spindexer {
  private SpindexerIO io;
  private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  private double requestedSpeed = -1;
  private boolean unjaming;

  public enum SpindexerStates {
    DISABLED,
    IDLE,
    INDEXING
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
        }
      }
      case DISABLED -> {}
    }

    Logger.recordOutput("Spindexer/State", state.toString());
    Logger.recordOutput("Spindexer/RequestedSpeed", requestedSpeed);
  }

  public void requestIdle() {
    state = SpindexerStates.IDLE;
    if (!unjaming) {
      requestedSpeed = 0;
    }
  }

  public void requestGoal(double speed) {
    state = SpindexerStates.INDEXING;
    if (!unjaming) {
      requestedSpeed = speed;
    }
  }

  public void unjamOverride(boolean unjaming) {
    this.unjaming = unjaming;
    requestedSpeed = Constants.Tunnel.unjamRPS;
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
