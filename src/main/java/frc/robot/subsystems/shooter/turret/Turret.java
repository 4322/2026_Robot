package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.util.ClockUtil;
import org.littletonrobotics.junction.Logger;

public class Turret {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private Double desiredDeg = 0.0;

  public enum turretState {
    DISABLED,
    SET_TURRET_ANGLE,
    UNWIND
  }

  public turretState state = turretState.SET_TURRET_ANGLE;

  public Turret(TurretIO io) {
    this.io = io;
    io.updateInputs(inputs);
    // Removed set postion code
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/State", state);
    Logger.recordOutput("Turret/needToUnwind", needsToUnwind());

    switch (Constants.turretMode) {
      case DISABLED -> {}
      case TUNING -> {}
      case NORMAL -> {
        switch (state) {
          case DISABLED -> {
            break;
          }
          case UNWIND -> {
            // Meant to unwind turret to a bound of mid point
            // This goes to the angle for the turret to unwind to
            if (desiredDeg != null) {
              io.setAngle(desiredDeg);
              if (MathUtil.isNear(inputs.turretDegs, Constants.Turret.midPointPhysicalDeg, 90)
                  && isAtGoal()) {
                state = turretState.SET_TURRET_ANGLE;
              }
            } else {
              io.setAngle(Constants.Turret.midPointPhysicalDeg);
            }
          }
          case SET_TURRET_ANGLE -> {
            if (desiredDeg != null) {
              io.setAngle(desiredDeg);
            } else {
              io.setAngle(Constants.Turret.midPointPhysicalDeg);
            }
          }
        }
      }
    }
  }

  public void requestAngle(Double angle, boolean safeToUnwind) {
    this.desiredDeg = angle;
    Logger.recordOutput("Turret/desiredDeg", desiredDeg);
    if (Constants.turretLocked) {
      return;
    }
    // Null represents a zone that returns no angle
    if (desiredDeg != null) {
      desiredDeg = getClosestTargetAngle(desiredDeg, inputs.turretDegs);
    } else {
      // In the case when we are in a zone that returns null angle
      desiredDeg = Constants.Turret.midPointPhysicalDeg;
      state = turretState.UNWIND;
    }

    // Code that is meant to set the degree of turret is unwind cases

    // setInMidpoint is a placehodler to reprsent a mod method
    // Meant to reduce desired degree to a number that is between +- 90 of mid

  }

  private double getTargetAngleInMidpoint() {
    Logger.recordOutput("Shooter/currentMethod", "getTargetAngleInMidpoint()");
    // The below code uses the desired angle and subtracts the midpoint
    // As to set the desired angle to reference 0 as to see the differnce between
    // The midpoint and desired angle so we can see how much angle we can reduce the angle by
    // as to get to the id
    if (Math.abs(desiredDeg - Constants.Turret.midPointPhysicalDeg) > 360) {
      double mod = ((desiredDeg - Constants.Turret.midPointPhysicalDeg) % 360) + 180;
      double modInverse = Math.abs(desiredDeg - mod) / 360;
      return (desiredDeg > 0) ? desiredDeg - (360 * modInverse) : desiredDeg + (360 * mod);
    } else {
      return (desiredDeg > 0) ? desiredDeg - 360 : desiredDeg + 360;
    }
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
    } else if (state == turretState.UNWIND) {
      return MathUtil.isNear(desiredDeg, inputs.turretDegs, 6);
    } else {
      return MathUtil.isNear(desiredDeg, inputs.turretDegs, Constants.Turret.goalToleranceDeg);
    }
  }

  public void setTurretAngleState() {
    state = turretState.SET_TURRET_ANGLE;
  }

  public void unwind(boolean safeToUnwind) {
    if (state == turretState.SET_TURRET_ANGLE && safeToUnwind) {
      desiredDeg =
          (MathUtil.isNear(Constants.Turret.midPointPhysicalDeg, desiredDeg, 90))
              ? desiredDeg
              : getTargetAngleInMidpoint();
      Logger.recordOutput("Turret/adjustedDeg", desiredDeg);
      state = turretState.UNWIND;
    } else if (!safeToUnwind) {
      state = turretState.SET_TURRET_ANGLE;
    }
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

  private double getClosestTargetAngle(double targetAngle, double currentAngle) {
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
