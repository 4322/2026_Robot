package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.Outake;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Hood {
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Hood/kP", Constants.Hood.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Hood/kI", Constants.Hood.kI);
  private static final LoggedTunableNumber kIZone =
      new LoggedTunableNumber("Hood/kIZone", Constants.Hood.kIZone);
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
  private boolean trenchOverride = false;
  private PIDController pidController = new PIDController(kP.get(), kI.get(), kD.get());
  private Timer hardwareTimer = new Timer();

  public Hood(HoodIO io) {
    this.io = io;
    pidController.disableContinuousInput();
    pidController.setIZone(kIZone.get());
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
            hashCode(), () -> pidController.setIZone(kIZone.get()), kIZone);
        LoggedTunableNumber.ifChanged(
            hashCode(), () -> pidController.setTolerance(toleranceDeg.get()), toleranceDeg);
        requestGoal(tuningGoalDeg.get());

        pidVelocity = pidController.calculate(inputs.degrees, requestedAngleDeg);

        io.setServoVelocity(pidVelocity);
        Logger.recordOutput("Hood/requestedServoVelocity", pidVelocity);
      }
      case NORMAL -> {
        if (Constants.currentMode == Constants.Mode.SIM && !homed) {
          io.simEstimatedPosition();
          homed = true;
        }
        if (!homed && DriverStation.isEnabled()) {
          io.setServoVelocity(Constants.Hood.homingVelocity);
          homingTimer.start();
          if (Math.abs(inputs.encoderRPS) > Constants.Hood.homingVelocityThresholdRPS) {
            homingTimer.reset();
          } else if (homingTimer.hasElapsed(0.1)) {
            io.setEncoderHomed();
            io.setServoVelocity(Constants.Hood.idleVelocity);
            homed = true;
            homingTimer.stop();
            homingTimer.reset();
          }
        } else if (DriverStation.isEnabled()) {
          pidVelocity = pidController.calculate(inputs.degrees, requestedAngleDeg);
          // let hood continually adjust, unless it is near the bottom, in which case
          // we don't want kI building up to max negative velocity
          if (pidController.atSetpoint() && requestedAngleDeg == Constants.Hood.safeAngleDeg) {
            pidVelocity = Constants.Hood.holdDownVelocity;
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
    Logger.recordOutput("Hood/Timer", homingTimer.get());
    Logger.recordOutput("Hood/homed", homed);
    Logger.recordOutput("Hood/isAtGoal", isAtGoal());
  }

  public void requestGoal(double angle) {
    if (!trenchOverride) {
      setGoal(angle);
    }
  }

  private void setGoal(double angle) {
    pidController.setSetpoint(angle);
    this.requestedAngleDeg = angle;
    Logger.recordOutput("Hood/goalDegree", requestedAngleDeg);
  }

  public void trenchOverride(boolean override) {
    setGoal(Constants.Hood.safeAngleDeg);
    trenchOverride = override;
  }

  public void rehome() {
    homed = false;
  }

  public boolean isAtGoal() {
    if (!homed || trenchOverride) {
      return false;
    } else if (Constants.currentMode == Constants.Mode.SIM) {
      return true; // TODO temporary until we get hood sim working
    } else {
      if (!pidController.atSetpoint()){
        hardwareTimer.start();
      } 
      
    if (hardwareTimer.hasElapsed(Outake.isScoring() ?Constants.Hood.scoringHardwareCheckTime: Constants.Hood.passingHardwareCheckTime)){
        hardwareTimer.stop();
        hardwareTimer.reset();
        return true;
      } else {
      return pidController.atSetpoint();
      }
    }
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
