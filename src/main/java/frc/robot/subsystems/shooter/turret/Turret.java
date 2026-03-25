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
  private double prevDeg = 0.0;
  private Timer hardwareTimer = new Timer();

  public enum turretState {
    DISABLED,
    SET_TURRET_ANGLE,
  }

  public turretState state = turretState.DISABLED;

  public Turret(TurretIO io) {
    this.io = io;
    // Temporary until we can,
    io.updateInputs(inputs);
    io.setPosition(getRotation());
    // Upper method sets the position of the turret in rotations
    // manual homing to the rear
  }

  public void inputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public void outputsPeriodic() {
    Logger.recordOutput("Turret/State", state);
    Logger.recordOutput("Turret/needToUnwind", needsToUnwind());
    Logger.recordOutput("Turret/atGoal", isAtGoal());

    switch (Constants.turretMode) {
      case DISABLED -> {}
      case TUNING -> {}
      case NORMAL -> {
        switch (state) {
          case DISABLED -> {
            if (!DriverStation.isDisabled()) {
              state = turretState.SET_TURRET_ANGLE;
            } else {
              io.setPosition(getRotation());
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
      }
    }
  }

  public void requestAngle(Double angle) {
    this.desiredDeg = angle;
    if (Constants.turretLocked) {
      return;
    } else if (state == turretState.DISABLED) {
      desiredDeg = getRotation();
      prevDeg = desiredDeg;
    }
    if (state == turretState.SET_TURRET_ANGLE) {
      if (desiredDeg != null) {
        desiredDeg = calculateAngle(desiredDeg, inputs.turretDegs);
        prevDeg = desiredDeg;
      } else {
        desiredDeg = prevDeg;
      }
    }
    Logger.recordOutput("Turret/desiredDeg", desiredDeg);
  }

  private double getTargetAngleInMidpoint() {
    Logger.recordOutput("Shooter/currentMethod", "getTargetAngleInMidpoint()");
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
      if (!MathUtil.isNear(desiredDeg, inputs.turretDegs, Constants.Turret.goalToleranceDeg)) {
        hardwareTimer.start();
      } else {
        hardwareTimer.stop();
        hardwareTimer.reset();
      }

      if (hardwareTimer.hasElapsed(
          shooter.isScoring()
              ? Constants.scoringHardwareCheckTime
              : Constants.passingHardwareCheckTime)) {
        return true;
      } else {
        return MathUtil.isNear(desiredDeg, inputs.turretDegs, Constants.Turret.goalToleranceDeg);
      }
    }
  }

  public void setTurretAngleState() {
    state = turretState.SET_TURRET_ANGLE;
  }

  public boolean atTurretAtUnwindLimit() {
    return MathUtil.isNear(Constants.Turret.midPointPhysicalDeg, desiredDeg, 180);
  }

  public boolean isUnwinding() {
    return needsToUnwind;
  }

  public void unwind(boolean needsUnwindFinish) {
    if (needsUnwindFinish) {
      desiredDeg =
          (MathUtil.isNear(Constants.Turret.midPointPhysicalDeg, desiredDeg, 90))
              ? desiredDeg
              : getTargetAngleInMidpoint();
    }
    needsToUnwind = needsUnwindFinish;
    prevDeg = desiredDeg;
    Logger.recordOutput("Turret/unwindDesiredDeg", prevDeg);
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

  private double getRotation() {
    if (state == turretState.DISABLED) {
      return 0.5;
    }
    // ----------------------
    // This is only in use due to the fact CRT is not working, aka the 0.5
    // ----------------------
    // Based off of 4522's "brute force" solver:
    // https://www.chiefdelphi.com/uploads/short-url/vvrM1V1pqvDnnZfHtAhS02mBVIi.pdf
    // This is only capable of calculating rotation in the 0-360 degree range.
    // This is because there's an ambiguity when the turret rotates 360 degrees; at one full
    // rotation both sensors will be at (0, 0) again.
    // Might be problematic in a brownout + over-rotated turret, but that's not here nor there.
    final int GCD = Constants.Turret.CANCoderOneRatio * Constants.Turret.CANCoderTwoRatio;
    double encoderOne = inputs.encoderOneCount / Constants.Turret.CANCoderResolution;
    double encoderTwo = inputs.encoderTwoCount / Constants.Turret.CANCoderResolution;

    double bestError = Double.MAX_VALUE;
    double turretFullRotations = 0.5;

    // look for the entry in both lists of possible values that is the same.
    for (int i = 0; i < GCD; i++) {
      // generate the candidate rotation value from encoder 1
      double candidate1 = mod(encoderOne + (double) i / Constants.Turret.CANCoderOneRatio, 1.0);
      for (int j = 0; j < GCD; j++) {
        // generate the candidate rotation value from encoder 2
        double candidate2 = mod((encoderTwo + (double) i / Constants.Turret.CANCoderTwoRatio), 1.0);
        double error = Math.abs(candidate2 - candidate1);
        if (error < bestError) {
          bestError = error;
          turretFullRotations = candidate1;
        }
      }
    }
    Logger.recordOutput("Turret/fullRotations", turretFullRotations);
    return turretFullRotations;

    // int CANCoderOneMod = mod(inputs.encoderOneCount, Constants.Turret.CANCoderTwoRatio);
    // int CANCoderTwoMod = mod(inputs.encoderTwoCount, Constants.Turret.CANCoderOneRatio);
    // double turretFullRotations = mod((10 * CANCoderOneMod) + (36 * CANCoderTwoMod), 45);
    // double turretFractionalRotations =
    //    inputs.encoderTwoCount
    //        / (double) Constants.Turret.CANCoderResolution
    //        / Constants.Turret.CANCoderTwoRatio;
    // Logger.recordOutput("Turret/fullRotations", turretFullRotations);
    // Logger.recordOutput("Turret/fractionalRotations", turretFractionalRotations);
    // return turretFullRotations + turretFractionalRotations;
  }

  private static double mod(double a, double b) {
    return ((a % b) + b) % b;
  }
}
