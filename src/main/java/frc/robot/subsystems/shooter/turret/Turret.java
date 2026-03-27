package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.util.ClockUtil;
import org.littletonrobotics.junction.Logger;

public class Turret {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private Double desiredDeg = 0.0;
  private boolean minInclusive = false;

  public enum turretState {
    DISABLED,
    SET_TURRET_ANGLE,
    UNWIND
  }

  public turretState state = turretState.DISABLED;

  public Turret(TurretIO io) {
    this.io = io;
    io.updateInputs(inputs);
    io.setPosition(getRotation() - 0.25); // adjust for offset locked (calibrated) position
  }

  public void inputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public void outputsPeriodic() {
    Logger.recordOutput("Turret/State", state);
    Logger.recordOutput("Turret/needToUnwind", needsToUnwind());
  }

  public void requestAngle(Double angle, boolean safeToUnwind) {
    this.desiredDeg = angle;
    Logger.recordOutput("Turret/desiredDeg", desiredDeg);
    if (Constants.turretLocked) {
      return;
    }
    double diffFromMid =
        Units.radiansToDegrees(
            MathUtil.angleModulus(
                Units.degreesToRadians(desiredDeg - Constants.Turret.midPointPhysicalDeg)));
    desiredDeg = Constants.Turret.midPointPhysicalDeg + diffFromMid;
    if (RobotContainer.intake.hasExtended()) {
      io.setAngle(desiredDeg);
    }

    /*
    if (desiredDeg != null) {
      if (needsToUnwind()) {
        state = turretState.UNWIND;
      }
    }
    if (desiredDeg != null) {
      if (inputs.turretDegs + 180 >= Constants.Turret.maxPhysicalLimitDeg) {
        minInclusive = true;
      } else if (inputs.turretDegs - 180 <= Constants.Turret.minPhysicalLimitDeg) {
        minInclusive = false;
      }
      desiredDeg = angleDistance(desiredDeg, inputs.turretDegs, minInclusive);
      if (state != turretState.UNWIND && !needsToUnwind()) {
        state = turretState.SET_TURRET_ANGLE;
      }
    }
    if (safeToUnwind && needsToUnwind() || desiredDeg == null) {
      desiredDeg = Constants.Turret.midPointPhysicalDeg;
    }
      */
    Logger.recordOutput("Turret/adjustedDeg", desiredDeg);
  }

  public boolean needsToUnwind() {
    return false; /*
                  return (inputs.turretDegs >= Constants.Turret.maxUnwindLimitDeg
                      || inputs.turretDegs <= Constants.Turret.minUnwindLimitDeg);*/
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
      return MathUtil.isNear(desiredDeg, inputs.turretDegs, Constants.Turret.goalToleranceDeg);
    }
  }

  public void setTurretAngleState() {
    state = turretState.SET_TURRET_ANGLE;
  }

  public void unwind() {
    desiredDeg = setAngleWithinMidpoint();
    state = turretState.UNWIND;
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

  private double angleDistance(double targetAngle, double currentAngle, boolean minInclusive) {
    double angleDistance =
        ClockUtil.inputModulus(targetAngle - currentAngle, -180, 180, minInclusive) + currentAngle;
    return angleDistance;
  }

  private double setAngleWithinMidpoint() {
    desiredDeg = desiredDeg == null ? Constants.Turret.midPointPhysicalDeg : desiredDeg;
    if (desiredDeg == Constants.Turret.midPointPhysicalDeg) {
      return desiredDeg;
    }
    return desiredDeg =
        MathUtil.isNear(Constants.Turret.midPointPhysicalDeg, desiredDeg, 180)
            ? desiredDeg
            : Constants.Turret.midPointPhysicalDeg;
  }

  private double getRotation() {
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
    Logger.recordOutput("Turret/rotationRange", ROTATIONAL_RANGE);
    Logger.recordOutput("Turret/fullRotations", turretFullRotations);
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
