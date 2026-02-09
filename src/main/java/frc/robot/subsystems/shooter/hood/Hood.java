package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import org.littletonrobotics.junction.Logger;

public class Hood {
  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private Drive drive;
  private double requestedAngleDEG = 0.0;
  private Timer homingTimer = new Timer();
  private double pastEncoderPosition = 0.0;
  private boolean homed = false;
  private PIDController pidController =
      new PIDController(Constants.Hood.kP, Constants.Hood.kI, Constants.Hood.kD);

  public enum HoodStates {
    DISABLED,
    HOMING,
    IDLE,
    SHOOTING
  }

  private HoodStates state = HoodStates.DISABLED;

  public Hood(HoodIO io) {

    this.io = io;
    pidController.setTolerance(Constants.Hood.hoodTolerance);
    pidController.disableContinuousInput();
    pidController.setPID(Constants.Hood.kP, Constants.Hood.kI, Constants.Hood.kD);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
    Logger.recordOutput("MEEEP", requestedAngleDEG);

    switch (state) {
      case DISABLED -> {
        if (DriverStation.isEnabled()) {
          state = HoodStates.HOMING;
        }
      }
      case HOMING -> {
        io.setServoVelocity(Constants.Hood.homingVelocity);
        homingTimer.start();
        if (Math.abs(inputs.rawRotations - pastEncoderPosition) < Constants.Hood.hoodTolerance
            || (Math.abs(inputs.rawRotations - pastEncoderPosition)
                > -Constants.Hood
                    .hoodTolerance)) { // We want to check if the encoder is at 0, but if the
          // encoder is disconnected it will always return 0, so we
          // also check if the timer has elapsed
          io.setEncoderHomed();
          homed = true;
          homingTimer.reset();
          homingTimer.stop();
          if (AreaManager.isShootingArea(drive.getPose().getTranslation())) {
            state = HoodStates.SHOOTING;
          } else {
            state = HoodStates.IDLE;
          }
        } else {
          pastEncoderPosition = inputs.rawRotations;
        }
      }
      case IDLE -> {
        requestIdle();
      }
      case SHOOTING -> {
        pidController.setSetpoint(requestedAngleDEG);
        if (pidController.atSetpoint()) {
          io.stopAt(requestedAngleDEG);
        } else {
          io.setServoVelocity((pidController.calculate(inputs.degrees, requestedAngleDEG)));
        }
      }
    }

    Logger.recordOutput("Hood/State", state.toString());
  }

  public void requestIdle() {
    state = HoodStates.IDLE;
    pidController.setSetpoint(Constants.Hood.idleDegrees);
    requestedAngleDEG = Constants.Hood.idleDegrees;
  }

  public void requestShoot(double angle) {
    if (state != HoodStates.HOMING && state != HoodStates.DISABLED) {
      state = HoodStates.SHOOTING;
       pidController.setSetpoint(angle);
      requestedAngleDEG = angle;
    }
  }

  public void rehome() {
    state = HoodStates.HOMING;
  }

  public boolean isAtGoal() {
    return pidController.atSetpoint();
  }

  public boolean isHomed() {
    return homed;
  }
}
