package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Hood {
  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private double requestedAngle = 0.0;
  private Timer homingTimer = new Timer();
  private double requestedHoodAngle = 0.0;

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
        io.setTargetAngle(-10.0); // Move to hard stop
        homingTimer.reset();
        homingTimer.start();
        if (homingTimer.hasElapsed(0.4)) {
          io.setHomedAngle(0.0); // Reset to home position
          homingTimer.stop();
          state = HoodStates.IDLE;
        }
      }
      case IDLE -> {
        requestIdle();
      }
      case SHOOTING -> {
        io.setTargetAngle(requestedAngle);
      }
    }

    Logger.recordOutput("Hood/State", state.toString());
  }

  public void requestIdle() {
    state = HoodStates.IDLE;
    io.setTargetAngle(Constants.Hood.idleDegrees);
    requestedAngle = Constants.Hood.idleDegrees;
  }

  public void requestShoot(double angle) {
    state = HoodStates.SHOOTING;
    requestedAngle = angle;
  }

  public void rehome() {
    state = HoodStates.HOMING;
  }

  public boolean isAtGoal() {
    return Math.abs(inputs.angleDeg - requestedAngle) < 1.0;
  }
}
