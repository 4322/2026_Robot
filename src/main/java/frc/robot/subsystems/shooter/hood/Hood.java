package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Hood {
  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private double requestedAngleDEG = 0.0;
  private Timer homingTimer = new Timer();
  private double pastEncoderPosition = 0.0;
  private double PIDCalculate;
  private boolean homed = false;
  private PIDController pidController =
      new PIDController(Constants.Hood.kP, Constants.Hood.kI, Constants.Hood.kD);

  public Hood(HoodIO io) {

    this.io = io;
    pidController.setTolerance(Constants.Hood.hoodTolerance);
    pidController.disableContinuousInput();
    pidController.setPID(Constants.Hood.kP, Constants.Hood.kI, Constants.Hood.kD);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
    Logger.recordOutput("Hood/requestedDegree", requestedAngleDEG);
    Logger.recordOutput("Hood/requestedServoVelocity", PIDCalculate);

    if (!homed && DriverStation.isEnabled()) {
      io.setServoVelocity(Constants.Hood.homingVelocity);
      homingTimer.start();
      if (homingTimer.hasElapsed(0.04)
          && Math.abs(inputs.encoderRPS) < Constants.Hood.homingVelocityThreshold) {
        io.setEncoderHomed();
        io.setServoVelocity(Constants.Hood.idleVelocity);
        homed = true;
        homingTimer.reset();
        homingTimer.stop();
      } else {
        if ((Math.abs(inputs.encoderRPS) > Constants.Hood.homingVelocityThreshold) || DriverStation.isDisabled()) {
          homingTimer.reset();
        }
        pastEncoderPosition = inputs.rawRotations;
      }
    }
    else if(DriverStation.isEnabled()){
    if (pidController.atSetpoint()) {
      io.setServoVelocity(0);
    } else {
      io.setServoVelocity((pidController.calculate(inputs.degrees, requestedAngleDEG)));
    }
    }
  }

  public void requestGoal(double angle) {
    pidController.setSetpoint(angle);
    requestedAngleDEG = angle;
    PIDCalculate = pidController.calculate(inputs.degrees, requestedAngleDEG);
  }

  public void rehome() {
    homed = false;
  }

  public boolean isAtGoal() {
    return pidController.atSetpoint();
  }

  public boolean isHomed() {
    return homed;
  }

  public double getEncoderDetectedPosition(){
    return inputs.degrees;
  }
}
