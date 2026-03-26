package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Hood {
  private static final LoggedTunableNumber fastVelocity =
      new LoggedTunableNumber("Hood/fastVelocity", Constants.Hood.fastVelocity);
  private static final LoggedTunableNumber mediumVelocity =
      new LoggedTunableNumber("Hood/mediumVelocity", Constants.Hood.mediumVelocity);
  private static final LoggedTunableNumber slowVelocity =
      new LoggedTunableNumber("Hood/slowVelocity", Constants.Hood.slowVelocity);
  private static final LoggedTunableNumber largeToleranceDeg =
      new LoggedTunableNumber("Hood/largeToleranceDeg", Constants.Hood.largeToleranceDeg);
  private static final LoggedTunableNumber mediumToleranceDeg =
      new LoggedTunableNumber("Hood/mediumToleranceDeg", Constants.Hood.mediumToleranceDeg);
  private static final LoggedTunableNumber smallToleranceDeg =
      new LoggedTunableNumber("Hood/smallToleranceDeg", Constants.Hood.smallToleranceDeg);
  private static final LoggedTunableNumber tuningGoalDeg =
      new LoggedTunableNumber("Hood/tuningGoalDeg", 0);
  private static final LoggedTunableNumber tuningPulseWidth =
      new LoggedTunableNumber("Hood/tuningpulseWidth", 0);

  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private double requestedAngleDeg;
  private Timer homingTimer = new Timer();
  private boolean homed = false;
  private boolean trenchOverride = false;
  private double lastVelocity = 0;

  public Hood(HoodIO io) {
    this.io = io;
  }

  public void inputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
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
          requestGoal(tuningGoalDeg.get());
        }
      }
      case NORMAL -> {
        if (Constants.currentMode == Constants.Mode.SIM && !homed) {
          io.simEstimatedPosition();
          homed = true;
        }
        if (!homed && DriverStation.isEnabled()) {
          io.setServoVelocity(Constants.Hood.homingVelocity, 0);
          homingTimer.start();
          if (Math.abs(inputs.encoderRPS) > Constants.Hood.homingVelocityThresholdRPS) {
            homingTimer.reset();
          } else if (homingTimer.hasElapsed(0.1)) {
            io.setEncoderHomed();
            io.setServoVelocity(0, 0);
            homed = true;
            homingTimer.stop();
            homingTimer.reset();
          }
        } else if (DriverStation.isEnabled()) {
          setVelocity();
        } else {
          io.setServoVelocity(0, 0);
          homingTimer.stop();
          homingTimer.reset();
        }
      }
    }
    Logger.recordOutput("Hood/Timer", homingTimer.get());
    Logger.recordOutput("Hood/homed", homed);
    Logger.recordOutput("Hood/isAtGoal", isAtGoal());
  }

  private void setVelocity() {
    double velocity = 0;
    if (Math.abs(inputs.degrees - requestedAngleDeg) < smallToleranceDeg.get()) {
      velocity = 0;
    } else if (inputs.degrees > requestedAngleDeg + largeToleranceDeg.get()
        || requestedAngleDeg == Constants.Hood.safeAngleDeg) {
      velocity = -fastVelocity.get();
    } else if (inputs.degrees < requestedAngleDeg - largeToleranceDeg.get()) {
      velocity = fastVelocity.get();
    } else if (inputs.degrees > requestedAngleDeg + mediumToleranceDeg.get()) {
      velocity = -mediumVelocity.get();
    } else if (inputs.degrees < requestedAngleDeg - mediumToleranceDeg.get()) {
      velocity = mediumVelocity.get();
    } else if (lastVelocity > 0) {
      if (inputs.degrees > requestedAngleDeg) {
        velocity = 0;
      } else {
        velocity = slowVelocity.get();
      }
    } else if (lastVelocity < 0) {
      if (inputs.degrees < requestedAngleDeg) {
        velocity = 0;
      } else {
        velocity = -slowVelocity.get();
      }
    }
    io.setServoVelocity(velocity, requestedAngleDeg);
    lastVelocity = velocity;
    Logger.recordOutput("Hood/requestedServoVelocity", velocity);
  }

  public void requestGoal(double angle) {
    if (!trenchOverride) {
      setGoal(angle);
    }
  }

  private void setGoal(double angle) {
    requestedAngleDeg = angle;
    setVelocity();
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
      return lastVelocity == 0;
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
