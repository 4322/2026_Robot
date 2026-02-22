package frc.robot.subsystems.shooter.tunnel;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Tunnel {
  private TunnelIO io;
  private TunnelIOInputsAutoLogged inputs = new TunnelIOInputsAutoLogged();
  private double requestedSpeed = -1;

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
    switch (Constants.tunnelMode) {
      case TUNING -> {}
      case NORMAL -> {
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
            io.setTargetMechanismRotations(requestedSpeed);
          }
        }
      }
    }

    Logger.recordOutput("Tunnel/State", state.toString());
    Logger.recordOutput("Tunnel/RequestedSpeed", requestedSpeed);
  }

  public void requestIdle() {
    state = TunnelStates.IDLE;
    requestedSpeed = 0;
  }

  public void requestIndex(double speed) {
    state = TunnelStates.INDEXING;
    requestedSpeed = speed;
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

  public double getVelocity() {
    return inputs.mechanismRotationsPerSec;
  }
}
