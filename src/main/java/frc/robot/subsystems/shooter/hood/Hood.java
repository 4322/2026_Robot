package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Hood {
  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private double requestedAngle = 0.0;
  private Timer homingTimer = new Timer();
  private double pastEncoderPosition = 0.0;
  private boolean homed = false;
  private PIDController pidController = new PIDController(Constants.Hood.kP, Constants.Hood.kI, Constants.Hood.kD);

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
        if (Math.abs(inputs.rawRotations - pastEncoderPosition) < Constants.Hood.homingThreshold || (Math.abs(inputs.rawRotations - pastEncoderPosition) > -Constants.Hood.homingThreshold)) { // We want to check if the encoder is at 0, but if the encoder is disconnected it will always return 0, so we also check if the timer has elapsed
          io.setEncoderPositionDEG(0);

          homingTimer.reset();
          homingTimer.stop();
          state = HoodStates.SHOOTING;
        }
        else {
            inputs.rawRotations = pastEncoderPosition;
        }
      
      }
      case IDLE -> {
        requestIdle();
      }
      case SHOOTING -> {
      pidController.setSetpoint(requestedAngle);
      if (pidController.atSetpoint()) {
      io.stopAt(requestedAngle);
      } else {
        io.setServoVelocity((pidController.calculate(inputs.rawRotations)));
      }
      }
    }

    Logger.recordOutput("Hood/State", state.toString());
  }

  public void requestIdle() {
    state = HoodStates.IDLE;
   pidController.setSetpoint(Constants.Hood.idleDegrees);
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
    return Math.abs(inputs.rawRotations - requestedAngle) < 1.0;
  }

  public boolean isHomed() {
    return homed;
  }
}
