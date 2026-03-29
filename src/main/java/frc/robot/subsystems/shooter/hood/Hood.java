package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
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
  private static final LoggedTunableNumber burstInterval =
      new LoggedTunableNumber("Hood/burstInterval", 0);

  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private double requestedAngleDeg;
  private Timer atGoalTimer = new Timer();
  private Timer homingTimer = new Timer();
  private int burstIntervalCount = 0;
  private boolean homed = false;
  private boolean trenchOverride = false;
  private double lastVelocity = 0;

  public Hood(HoodIO io) {
    this.io = io;
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
        if (tuningPulseWidth.get() == 0) {
          setGoal(tuningGoalDeg.get());
          setVelocity();
        } else if (burstInterval.getAsDouble() >= burstIntervalCount++) {
          burstIntervalCount = 0;
          io.setPulseWidth((int) tuningPulseWidth.get());
        } else {
          io.setPulseWidth(1500); // pause output
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
        updateNetworkTableValues();
      }
    }
    Logger.recordOutput("Shooter/Hood/Timer", homingTimer.get());
    Logger.recordOutput("Shooter/Hood/homed", homed);
    Logger.recordOutput("Shooter/Hood/isAtGoal", isAtGoal());
    Logger.recordOutput("Shooter/Hood/goalDegrees", requestedAngleDeg);
  }

  private void setVelocity() {
    double velocity = 0;
    updateAtGoalTimer();
    if (Math.abs(inputs.degrees - requestedAngleDeg) < smallToleranceDeg.get()) {
      velocity = 0;
    } else if (inputs.degrees > requestedAngleDeg + largeToleranceDeg.get()
        || (requestedAngleDeg == Constants.Hood.safeAngleDeg
            && inputs.degrees > Constants.Hood.safeAngleDeg)) {
      velocity = -fastVelocity.get();
    } else if (inputs.degrees < requestedAngleDeg - largeToleranceDeg.get()) {
      velocity = fastVelocity.get();
    } else if (inputs.degrees > requestedAngleDeg + mediumToleranceDeg.get()) {
      velocity = -mediumVelocity.get();
    } else if (inputs.degrees < requestedAngleDeg - mediumToleranceDeg.get()) {
      velocity = mediumVelocity.get();
    } else {
      if (lastVelocity > 0) {
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
    }
    if (burstInterval.getAsDouble() >= burstIntervalCount++) {
      burstIntervalCount = 0;
      io.setServoVelocity(velocity, requestedAngleDeg);
      lastVelocity = velocity;
    } else {
      io.setServoVelocity(0, 0);
    }
    Logger.recordOutput("Shooter/Hood/requestedServoVelocity", velocity);
  }

  private void updateAtGoalTimer() {
    if (Math.abs(inputs.degrees - requestedAngleDeg) < mediumToleranceDeg.get()) {
      atGoalTimer.start();
    } else {
      atGoalTimer.stop();
      atGoalTimer.reset();
    }
  }

  private void updateNetworkTableValues() {
    if (isAtGoal()) {
      if (lastVelocity == 0) {
        SmartDashboard.putString("Hood/HoodAtGoal", Constants.NetworkTables.green.toHexString());
      } else {
        SmartDashboard.putString("Hood/HoodAtGoal", Constants.NetworkTables.yellow.toHexString());
      }
    } else {
      SmartDashboard.putString("Hood/HoodAtGoal", Constants.NetworkTables.red.toHexString());
    }
    SmartDashboard.putNumber("Hood/HoodAngle", inputs.degrees);
  }

  public void requestGoal(double degrees) {
    if (Constants.hoodMode == SubsystemMode.NORMAL && !trenchOverride) {
      setGoal(degrees);
    }
  }

  private void setGoal(double degrees) {
    requestedAngleDeg = degrees;
    updateAtGoalTimer();
  }

  private void servoSeekPosition(double hoodDegrees) {
    double servoDegrees =
        hoodDegrees
            * Constants.Hood.encoderToHoodGearRatio
            * Constants.Hood.servoToEncoderGearRatio;
    int pulseWdith =
        MathUtil.clamp(
            500 + Constants.Hood.homeMicroSecOffset + (int) (servoDegrees * 0.9), 500, 2500);
    io.setPulseWidth(pulseWdith);
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
    } else if (lastVelocity == 0 || atGoalTimer.hasElapsed(Constants.Hood.atGoalTimeoutSec)) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isHomed() {
    return homed;
  }

  public double getPositionDegrees() {
    return inputs.degrees;
  }

  public void enableBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }
}
