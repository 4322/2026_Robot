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
  private static final LoggedTunableNumber kIZone =
      new LoggedTunableNumber("Hood/kIZone", Constants.Hood.kIZone);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Hood/kD", Constants.Hood.kD);
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hood/toleranceDeg", Constants.Hood.toleranceDeg);
  private static final LoggedTunableNumber tuningGoalDeg =
      new LoggedTunableNumber("Hood/tuningGoalDeg", 0);
  private static final LoggedTunableNumber tuningPulseWidth =
      new LoggedTunableNumber("Hood/tuningpulseWidth", 0);

  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private double requestedAngleDeg;
  private Timer homingTimer = new Timer();
  private double pidVelocity;
  private boolean homed = false;
  private boolean trenchOverride = false;
  private PIDController pidController = new PIDController(kP.get(), kI.get(), kD.get());

  public Hood(HoodIO io) {
    this.io = io;
    pidController.disableContinuousInput();
    pidController.setIZone(kIZone.get());
    pidController.setTolerance(toleranceDeg.get());
  }

  public void inputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Hood", inputs);
  }

  public void outputsPeriodic() {
    switch (Constants.hoodMode) {
      case DISABLED -> {
        homed = true;
      }
      case TUNING -> {
        if (!homed) {
          io.setEncoderHomed();
          homed = true;
        }
        if (tuningPulseWidth.get() != 0) {
          io.setPulseWidth((int) tuningPulseWidth.get());
        } else {
          LoggedTunableNumber.ifChanged(
              hashCode(), () -> pidController.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
          LoggedTunableNumber.ifChanged(
              hashCode(), () -> pidController.setIZone(kIZone.get()), kIZone);
          LoggedTunableNumber.ifChanged(
              hashCode(), () -> pidController.setTolerance(toleranceDeg.get()), toleranceDeg);
          requestGoal(tuningGoalDeg.get());

          pidVelocity = pidController.calculate(inputs.degrees, requestedAngleDeg);

          io.setServoVelocity(pidVelocity);
          Logger.recordOutput("Shooter/Hood/requestedServoVelocity", pidVelocity);
        }
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
          Logger.recordOutput("Shooter/Hood/requestedServoVelocity", pidVelocity);
        } else {
          io.setServoVelocity(Constants.Hood.idleVelocity);
          homingTimer.stop();
          homingTimer.reset();
        }
      }
    }
    Logger.recordOutput("Shooter/Hood/Timer", homingTimer.get());
    Logger.recordOutput("Shooter/Hood/homed", homed);
    Logger.recordOutput("Shooter/Hood/isAtGoal", isAtGoal());
  }

  public void requestGoal(double angle) {
    if (!trenchOverride) {
      setGoal(angle);
    }
  }

  private void setGoal(double angle) {
    pidController.setSetpoint(angle);
    this.requestedAngleDeg = angle;
    Logger.recordOutput("Shooter/Hood/goalDegree", requestedAngleDeg);
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
      return pidController.atSetpoint();
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
