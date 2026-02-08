package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

public class Hood {
  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private double requestedAngle = 0.0;

  public enum HoodStates {
    DISABLED,
    HOMING,
    IDLE,
    SHOOTING
  }

  private HoodStates state = HoodStates.DISABLED;

  public Hood(HoodIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    switch (state) {
      case DISABLED -> {
        if (DriverStation.isEnabled()) {
          state = HoodStates.HOMING;
        }
      }
      case HOMING -> {
        // TODO homing logic
      }
      case IDLE -> {}
      case SHOOTING -> {}
    }

    Logger.recordOutput("Hood/State", state.toString());
  }

  public void requestIdle() {
    state = HoodStates.IDLE;
  }

  public void requestShoot(double angle) {
    state = HoodStates.SHOOTING;
    requestedAngle = angle;
  }
}
