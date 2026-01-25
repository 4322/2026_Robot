package frc.robot.subsystems.shooter.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  private SpindexerIO io;
  private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  private boolean requestIdle;
  private boolean requestIndex;

  public enum SpindexerStates {
    DISABLED,
    IDLE,
    INDEXING
  }

  private SpindexerStates state = SpindexerStates.DISABLED;

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
    Logger.recordOutput("Spindexer/State", state.toString());

    switch (state) {
      case DISABLED -> {
        if (requestIdle) {
          state = SpindexerStates.IDLE;
          unsetAllRequests();
        }
      }
      case IDLE -> {
        io.stop();
        if (requestIndex) {
          state = SpindexerStates.INDEXING;
          unsetAllRequests();
        }
      }
      case INDEXING -> {
        io.setTargetVelocity(Constants.Spindexer.indexingVelocityRotationsPerSec);
        if (requestIdle) {
          state = SpindexerStates.IDLE;
          unsetAllRequests();
        }
      }
    }
  }

  private void unsetAllRequests() {
    requestIdle = false;
    requestIndex = false;
  }
}
