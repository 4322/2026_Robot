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
       io.homingPulseWidth();
       
        homingTimer.start();
        if (homingTimer.hasElapsed(0.4) || Math.abs(inputs.rotations) > 0.5) { 
          io.setEncoderPosition(0);
          homingTimer.reset();
          homingTimer.stop();
          state = HoodStates.SHOOTING;
        }
      }
      case IDLE -> {
        requestIdle();
      }
      case SHOOTING -> {

      }
    }

    Logger.recordOutput("Hood/State", state.toString());
  }

  public void requestIdle() {
    state = HoodStates.IDLE;
    // io.setServoPosition(Constants.Hood.idleDegrees);
    requestedAngle = Constants.Hood.idleDegrees;
  }

  public void requestShoot(double angle) {
    if (state != HoodStates.HOMING && state != HoodStates.DISABLED) {
      state = HoodStates.SHOOTING;
      requestedAngle = angle;
    }
    
  }

  public void rehome() {
    state = HoodStates.HOMING;
  }

  public boolean isAtGoal() {
    return Math.abs(inputs.rotations - requestedAngle) < 1.0;
  }
}
