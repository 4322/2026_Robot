package frc.robot.subsystems.shooter.tunnel;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Tunnel {
  private TunnelIO io;
  private TunnelIOInputsAutoLogged inputs = new TunnelIOInputsAutoLogged();

  public enum TunnelStates {
    DISABLED,
    IDLE,
    INDEXING
  }

  private TunnelStates state = TunnelStates.DISABLED;

  public Tunnel(TunnelIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Tunnel", inputs);

    switch (state) {
      case DISABLED -> {
        if (DriverStation.isEnabled()) {
          state = TunnelStates.IDLE;
        }
      }
      case IDLE -> {
        io.stop();
      }
      case INDEXING -> {
        io.setTargetMechanismRotations(Constants.Tunnel.indexingMotorRotationsPerSec);
      }
    }

    Logger.recordOutput("Tunnel/State", state.toString());
  }

  public void requestIdle() {
    state = TunnelStates.IDLE;
  }

  public void requestIndex() {
    state = TunnelStates.INDEXING;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public boolean isStopped() {
    return inputs.mechanismRotationsPerSec < Constants.Tunnel.stoppedMechanismRotationsPerSec;
  }

  public boolean isAtSpeed() {
    return inputs.mechanismRotationsPerSec > Constants.Tunnel.atSpeedMechanismRotationsPerSec;
  }
}
