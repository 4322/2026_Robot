package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.util.ClockUtil;
import org.littletonrobotics.junction.Logger;

public class Turret {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  public boolean needsToUnwind = false;
  private Double desiredDeg = 0.0;
  private Double azimuth = 0.0;
  private double prevDeg = 0.0;
  private Timer atGoalTimer = new Timer();
  private boolean isScoring = false;

  public enum turretState {
    DISABLED,
    SET_TURRET_ANGLE,
  }

  public turretState state = turretState.DISABLED;

  public Turret(TurretIO io) {
    this.io = io;
    // Temporary until we can,
    // Temporary until we can,
    io.updateInputs(inputs);
    io.setPosition(getRotation() - 0.25); // adjust for offset locked (calibrated) position
  }

  public void inputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Turret", inputs);
  }

  public void outputsPeriodic() {
    switch (Constants.turretMode) {
      case DISABLED -> {}
      case TUNING -> {}
      case NORMAL -> {
        updateAtGoalTimer();

        if (DriverStation.isDisabled()) {
          state = turretState.DISABLED;
        }

        switch (state) {
          case DISABLED -> {
            // If ever disabled during unwind, ensure variable is reset
            // Ensure turret remains in position upon re-enabling after moving while disabled
            desiredDeg = inputs.turretDegs;
            prevDeg = inputs.turretDegs;
            needsToUnwind = false;
            if (!DriverStation.isDisabled()) {
              state = turretState.SET_TURRET_ANGLE;
            }
            break;
          }
          case SET_TURRET_ANGLE -> {
            if (desiredDeg != null) {
              io.setAngle(desiredDeg);
            } else {
              io.setAngle(prevDeg);
            }
          }
        }

        Logger.recordOutput("Shooter/Turret/State", state);
        Logger.recordOutput("Shooter/Turret/needToUnwind", needsToUnwind());
        Logger.recordOutput("Shooter/Turret/isUnwinding", isUnwinding());
        Logger.recordOutput("Shooter/Turret/atGoal", isAtGoal());
        Logger.recordOutput("Shooter/Turret/desiredDeg", desiredDeg);
      }
    }
  }

  public void requestAngle(Double angle, Boolean isScoring) {
    this.desiredDeg = angle;
    azimuth = desiredDeg;
    if (Constants.turretLocked) {
      return;
    }

    if (state == turretState.SET_TURRET_ANGLE) {
      if (desiredDeg != null) {
        desiredDeg = calculateAngle(desiredDeg, inputs.turretDegs);
        if (needsToUnwind()) {
          desiredDeg =
              MathUtil.clamp(
                  desiredDeg,
                  Constants.Turret.minPhysicalLimitDeg,
                  Constants.Turret.maxPhysicalLimitDeg);
        }
        prevDeg = desiredDeg;
      } else {
        desiredDeg = prevDeg;
      }
    }
    this.isScoring = isScoring;
  }

  private double getTargetAngleInMidpoint() {
    Logger.recordOutput("Shooter/Turret/currentMethod", "getTargetAngleInMidpoint()");
    return (desiredDeg - Constants.Turret.midPointPhysicalDeg) > 0
        ? desiredDeg - 360
        : (desiredDeg - Constants.Turret.midPointPhysicalDeg) < 0 ? desiredDeg + 360 : desiredDeg;
  }

  public boolean needsToUnwind() {
    return (inputs.turretDegs >= Constants.Turret.maxUnwindLimitDeg
        || inputs.turretDegs <= Constants.Turret.minUnwindLimitDeg);
  }

  public boolean isAtGoal() {
    if (Constants.turretLocked) {
      // desired turret angle is required robot heading when turret is locked
      return MathUtil.isNear(
          desiredDeg,
          RobotContainer.drive.getRotation().getDegrees(),
          Constants.Turret.goalToleranceLockedDeg);
    } else if (Constants.turretMode == Constants.SubsystemMode.DISABLED) {
      return true;
    } else {
      return Math.abs(inputs.turretDegs - desiredDeg) < Constants.Turret.smallToleranceDeg
          || (isScoring
              ? atGoalTimer.hasElapsed(Constants.scoringDoubleToleranceTime)
              : atGoalTimer.hasElapsed(Constants.passingDoubleToleranceTime));
    }
  }

  public void setTurretAngleState() {
    state = turretState.SET_TURRET_ANGLE;
  }

  public boolean atUnwindLimit() {
    return MathUtil.isNear(Constants.Turret.midPointPhysicalDeg, inputs.turretDegs, 180.1);
  }

  public boolean isUnwinding() {
    return this.needsToUnwind;
  }

  public void unwind(boolean needsUnwindFinish) {
    if (needsUnwindFinish) {
      if (needsToUnwind()) {
        desiredDeg = calculateAngle(azimuth, inputs.turretDegs);
      }
      desiredDeg =
          (MathUtil.isNear(Constants.Turret.midPointPhysicalDeg, desiredDeg, 180.1))
              ? desiredDeg
              : getTargetAngleInMidpoint();
    }
    this.needsToUnwind = needsUnwindFinish;
    prevDeg = desiredDeg;
    Logger.recordOutput("Shooter/Turret/unwindDesiredDeg", prevDeg);
  }

  public double getAngle() {
    if (Constants.turretLocked) {
      return 90;
    } else {
      return inputs.turretDegs;
    }
  }

  public void setBrakeMode(Boolean mode) {
    io.setBrakeMode(mode);
  }

  private double calculateAngle(double targetAngle, double currentAngle) {
    // Sets minInclusive based on desired degree aka if +-180 % 180, the remainder is 0
    // So if it's 0 we don't know our desired angle, since it could +- 180
    // Then to fix this minInclusive is evaluated if our difference between our target angle
    // and current angle is +- 180 and go to it's respective angle
    boolean minInclusive;
    if (targetAngle - currentAngle == 180) {
      minInclusive = false;
    } else {
      minInclusive = true;
    }
    // The input modulus equation is used to find the distance to closest angle matching the target
    // angle based on
    // the current angle
    // and then add that distance of the current angle to set the target to the closest target
    // angle.
    targetAngle =
        ClockUtil.inputModulus(targetAngle - currentAngle, -180, 180, minInclusive) + currentAngle;

    return targetAngle;
  }

  private void updateAtGoalTimer() {
    if (Math.abs(inputs.turretDegs - desiredDeg) < Constants.Turret.largeToleranceDeg) {
      atGoalTimer.start();
    } else {
      atGoalTimer.stop();
      atGoalTimer.reset();
    }
  }

  private double getRotation() {
    if (state == turretState.DISABLED) {
      return 0.5;
    }
    // ----------------------
    // This is only in use due to the fact CRT is not working, aka the 0.5
    // ----------------------    
    
    // returns rotations relative to locked position

    // Based off of 4522's "brute force" solver:
    // https://www.chiefdelphi.com/uploads/short-url/vvrM1V1pqvDnnZfHtAhS02mBVIi.pdf

    // HACK: Look. I know this value works, okay?
    final int GCD = 90;
    final double LCM = lcm(Constants.Turret.CANCoderOneRatio, Constants.Turret.CANCoderTwoRatio);
    // Range of motion in rotations
    final double ROTATIONAL_RANGE =
        LCM / Constants.Turret.CANCoderOneRatio / Constants.Turret.CANCoderTwoRatio;

    double encoderOne = inputs.encoderOneRot;
    double encoderTwo = inputs.encoderTwoRot;

    double bestError = Double.MAX_VALUE;
    double turretFullRotations = 0.5;

    // look for the entry in both lists of possible values that is the same.
    for (int i = 0; i < GCD; i++) {
      // generate the candidate rotation value from encoder 1
      double candidate1 =
          mod((encoderOne + (double) i) / Constants.Turret.CANCoderOneRatio, ROTATIONAL_RANGE);
      for (int j = 0; j < GCD; j++) {
        // generate the candidate rotation value from encoder 2
        double candidate2 =
            mod(((encoderTwo + (double) j) / Constants.Turret.CANCoderTwoRatio), ROTATIONAL_RANGE);
        double error = Math.abs(candidate2 - candidate1);
        if (error < bestError) {
          bestError = error;
          turretFullRotations = candidate1;
        }
      }
    }
    Logger.recordOutput("Shooter/Turret/rotationRange", ROTATIONAL_RANGE);
    Logger.recordOutput("Shooter/Turret/fullRotations", turretFullRotations);
    return turretFullRotations;
  }

  private static double mod(double a, double b) {
    return ((a % b) + b) % b;
  }

  /**
   * Least common multiple that works on fractional values.
   *
   * @param a value a
   * @param b value b
   * @return least common multiple
   */
  private static double lcm(double a, double b) {
    double i = 1.0;
    while (true) {
      if (a * i % b < 0.001) {
        return a * i;
      }
      i += 1.0;
    }
  }
}
