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
  private boolean isScoring = false;

  public Hood(HoodIO io) {
    this.io = io;
    moveServoToPosition(0); // avoid initial pop-up of servo when powered on
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
          homeHood();
        } else if (tuningPulseWidth.get() == 0) {
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
        if (!homed) {
          homeHood();
        } else {
          moveServoToPosition(requestedAngleDeg);
          updateAtGoalTimer();
        }
        updateNetworkTableValues();
      }
    }
    Logger.recordOutput("Shooter/Hood/Timer", homingTimer.get());
    Logger.recordOutput("Shooter/Hood/homed", homed);
    Logger.recordOutput("Shooter/Hood/isAtGoal", isAtGoal());
    Logger.recordOutput("Shooter/Hood/goalDegrees", requestedAngleDeg);
  }

  private void homeHood() {
    if (!DriverStation.isEnabled()) {
      homingTimer.stop();
      homingTimer.reset();
    } else {
      moveServoToPosition(0);
      homingTimer.start();
      if (homingTimer.hasElapsed(Constants.Hood.minHomingSec)
          && Math.abs(inputs.encoderRPS) <= Constants.Hood.homingVelocityThresholdRPS) {
        io.setEncoderHomed();
        setGoal(0);
        homed = true;
        homingTimer.stop();
        homingTimer.reset();
      }
    }
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

  public void requestGoal(double degrees, boolean isScoring) {
    this.isScoring = isScoring;
    if (Constants.hoodMode == SubsystemMode.NORMAL && !trenchOverride) {
      setGoal(degrees);
    }
  }

  private void setGoal(double degrees) {
    requestedAngleDeg = degrees;
    updateAtGoalTimer();
  }

  private void moveServoToPosition(double hoodDegrees) {
    double servoScaleFactor =
        Constants.Hood.servoLowPositionScaleFactor
            + ((hoodDegrees - Constants.Hood.lowCalibrationDeg)
                    / (Constants.Hood.highCalibrationDeg - Constants.Hood.lowCalibrationDeg))
                * (Constants.Hood.servoHighPositionScaleFactor
                    - Constants.Hood.servoLowPositionScaleFactor);
    double servoDegrees =
        hoodDegrees
            * Constants.Hood.encoderToHoodGearRatio
            * Constants.Hood.servoToEncoderGearRatio;
    double pulseWidthToDegreeRatio = 0.9;
    int pulseWdith =
        MathUtil.clamp(
            Constants.Hood.homePulseWidth
                + (int) (servoDegrees / pulseWidthToDegreeRatio * servoScaleFactor),
            500,
            2500);
    io.setPulseWidth(pulseWdith);
  }

  public void trenchOverride(boolean override) {
    setGoal(Constants.Hood.safeAngleDeg);
    trenchOverride = override;
  }

  public boolean isSafeAngle() {
    return inputs.hoodDegrees < Constants.Hood.safeAngleDeg + Constants.Hood.largeToleranceDeg;
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
        || (isScoring
            ? atGoalTimer.hasElapsed(Constants.scoringDoubleToleranceTime)
            : atGoalTimer.hasElapsed(Constants.passingDoubleToleranceTime))) {
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
}
