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
  private static final LoggedTunableNumber largeToleranceDeg =
      new LoggedTunableNumber("Hood/largeToleranceDeg", Constants.Hood.largeToleranceDeg);
  private static final LoggedTunableNumber smallToleranceDeg =
      new LoggedTunableNumber("Hood/smallToleranceDeg", Constants.Hood.smallToleranceDeg);
  private static final LoggedTunableNumber tuningGoalDeg =
      new LoggedTunableNumber("Hood/tuningGoalDeg", 0);
  private static final LoggedTunableNumber tuningPulseWidth =
      new LoggedTunableNumber("Hood/tuningpulseWidth", 0);
  private static final LoggedTunableNumber tuningBurstInterval =
      new LoggedTunableNumber("Hood/tuningBurstInterval", 0);

  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private double requestedAngleDeg;
  private Timer atGoalTimer = new Timer();
  private Timer homingTimer = new Timer();
  private int burstIntervalCount = 0;
  private boolean homed = false;
  private boolean trenchOverride = false;

  public Hood(HoodIO io) {
    this.io = io;
  }

  public void inputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Hood", inputs);
  }

  public void outputsPeriodic() {
    updateAtGoalTimer();
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
          moveServoToPosition(requestedAngleDeg);
        } else if (++burstIntervalCount > tuningBurstInterval.getAsDouble()) {
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
          io.setPulseWidth(500);
          homingTimer.start();
          if (Math.abs(inputs.encoderRPS) > Constants.Hood.homingVelocityThresholdRPS) {
            homingTimer.reset();
          } else if (homingTimer.hasElapsed(0.1)) {
            io.setEncoderHomed();
            setGoal(0);
            homed = true;
            homingTimer.stop();
            homingTimer.reset();
          }
        } else if (DriverStation.isEnabled()) {
          moveServoToPosition(requestedAngleDeg);
          updateAtGoalTimer();
        } else {
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

  private void updateAtGoalTimer() {
    if (Math.abs(inputs.hoodDegrees - requestedAngleDeg) < largeToleranceDeg.get()) {
      atGoalTimer.start();
    } else {
      atGoalTimer.stop();
      atGoalTimer.reset();
    }
  }

  private void updateNetworkTableValues() {
    if (isAtGoal()) {
      if (isWithinSmallTolerance()) {
        SmartDashboard.putString("Hood/HoodAtGoal", Constants.NetworkTables.green.toHexString());
      } else {
        SmartDashboard.putString("Hood/HoodAtGoal", Constants.NetworkTables.yellow.toHexString());
      }
    } else {
      SmartDashboard.putString("Hood/HoodAtGoal", Constants.NetworkTables.red.toHexString());
    }
    SmartDashboard.putNumber("Hood/HoodDegrees", inputs.hoodDegrees);
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

  private void moveServoToPosition(double hoodDegrees) {
    double servoDegrees =
        hoodDegrees
            * Constants.Hood.encoderToHoodGearRatio
            * Constants.Hood.servoToEncoderGearRatio;
    int pulseWdith =
        MathUtil.clamp(Constants.Hood.homePulseWidth + (int) (servoDegrees * 0.9), 500, 2500);
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
    } else if (isWithinSmallTolerance()
        || atGoalTimer.hasElapsed(Constants.Hood.atGoalTimeoutSec)) {
      return true;
    } else {
      return false;
    }
  }

  private boolean isWithinSmallTolerance() {
    return Math.abs(inputs.hoodDegrees - requestedAngleDeg) < smallToleranceDeg.get();
  }

  public boolean isHomed() {
    return homed;
  }

  public double getPositionDegrees() {
    return inputs.hoodDegrees;
  }

  public void enableBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }
}
