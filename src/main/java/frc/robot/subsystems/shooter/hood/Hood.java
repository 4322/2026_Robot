package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Hood {
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Hood/kP", Constants.Hood.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Hood/kI", Constants.Hood.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Hood/kD", Constants.Hood.kD);
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hood/toleranceDeg", Constants.Hood.toleranceDeg);
  private static final LoggedTunableNumber tuningGoalDeg =
      new LoggedTunableNumber("Hood/tuningGoalDeg", 0);

  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private double requestedAngleDeg;
  private Timer homingTimer = new Timer();
  private double pidVelocity;
  private boolean homed = false;
  private PIDController pidController = new PIDController(kP.get(), kI.get(), kD.get());

  public Hood(HoodIO io) {
    this.io = io;
    pidController.disableContinuousInput();
    pidController.setTolerance(toleranceDeg.get());
  }

  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    switch (Constants.hoodMode) {
      case DISABLED -> {
        homed = true;
      }
      case TUNING -> {
        if (!homed) {
          io.setEncoderHomed();
          homed = true;
        }
        LoggedTunableNumber.ifChanged(
            hashCode(), () -> pidController.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
            hashCode(), () -> pidController.setTolerance(toleranceDeg.get()), toleranceDeg);
        requestGoal(tuningGoalDeg.get());

        pidVelocity = pidController.calculate(inputs.degrees, requestedAngleDeg);
        // if (pidController.atSetpoint()) {
        //   pidVelocity = 0;
        // }
        io.setServoVelocity(pidVelocity);
        Logger.recordOutput("Hood/requestedServoVelocity", pidVelocity);
      }
      case NORMAL -> {
        if (!homed && DriverStation.isEnabled()) {
          io.setServoVelocity(Constants.Hood.homingVelocity);
          homingTimer.start();
          if (Math.abs(inputs.encoderRPS) > Constants.Hood.homingVelocityThresholdRPS) {
            homingTimer.reset();
          } else if (homingTimer.hasElapsed(0.04)) {
            io.setEncoderHomed();
            io.setServoVelocity(Constants.Hood.idleVelocity);
            homed = true;
            homingTimer.stop();
            homingTimer.reset();
          }
        } else if (DriverStation.isEnabled()) {
          pidVelocity = pidController.calculate(inputs.degrees, requestedAngleDeg);
          if (pidController.atSetpoint()) {
            pidVelocity = 0;
          }
          io.setServoVelocity(pidVelocity);
          Logger.recordOutput("Hood/requestedServoVelocity", pidVelocity);
        } else {
          io.setServoVelocity(Constants.Hood.idleVelocity);
          homingTimer.stop();
          homingTimer.reset();
        }
      }
    }
    Logger.recordOutput("Hood/homed", homed);
  }

  public void requestGoal(double angle) {
    pidController.setSetpoint(angle);
    this.requestedAngleDeg = angle;
    Logger.recordOutput("Hood/requestedDegree", requestedAngleDeg);
  }

  public void rehome() {
    homed = false;
  }

  public boolean isAtGoal() {
    // TODO temporary until we get hood sim working
    if (Constants.simMode == Constants.Mode.SIM) {
      return true;
    }
    return pidController.atSetpoint();
  }

  public boolean isHomed() {
    return homed;
  }

  public double getEncoderDetectedPosition() {
    return inputs.degrees;
  }

  public void enableBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }
}
