package frc.robot.subsystems.shooter.tunnel;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Tunnel {
  private TunnelIO io;
  private TunnelIOInputsAutoLogged inputs = new TunnelIOInputsAutoLogged();
  private double requestedSpeed = -1;
  private boolean unjamOverride;

  public enum TunnelStates {
    DISABLED,
    IDLE,
    INDEXING,
    UNJAM
  }

  private TunnelStates statee = TunnelStates.DISABLED;

  public Tunnel(TunnelIO io) {
    this.io = io;
  }

  public void inputsPeriodic() {

  }

  public void outputsPeriodic() {
    switch (Constants.tunnelMode) {
      case TUNING -> {}
      case DISABLED -> {}
      case NORMAL -> {
        if (DriverStation.isDisabled()) {
          statee = TunnelStates.DISABLED;
        }

        switch (statee) {
          case DISABLED -> {
            // Reset variables
            unjamOverride = false;
            if (DriverStation.isEnabled()) {
              statee = TunnelStates.IDLE;
            }
          }
          case IDLE -> {
            io.stop();
          }
          case INDEXING -> {
            io.setTargetMechanismRotations(requestedSpeed);
          }
          case UNJAM -> {
            io.setTargetMechanismRotations(Constants.Tunnel.unjamRPS);
          }
        }
      }
    }

   
  }

  public void requestIdle() {
    if (!unjamOverride) {
      statee = TunnelStates.IDLE;
      requestedSpeed = 0;
    }
  }

  public void unjamOverride(boolean unjamOverride) {
    this.unjamOverride = unjamOverride;
    if (unjamOverride) {
      statee = TunnelStates.UNJAM;
      requestedSpeed = Constants.Tunnel.unjamRPS;
    } else {
      // Default to idle when unjam isn't desired
      statee = TunnelStates.IDLE;
      requestedSpeed = 0;
    }
  }

  public void requestGoal(double speed) {
    if (!unjamOverride) {
      statee = TunnelStates.INDEXING;
      requestedSpeed = speed;
    }
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public boolean isStopped() {
    return inputs.mechanismRPS < Constants.Tunnel.stoppedMechanismRotationsPerSec;
  }

  public double getVelocity() {
    return inputs.mechanismRPS;
  }
}
