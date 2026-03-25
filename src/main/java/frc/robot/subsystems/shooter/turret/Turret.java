package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ClockUtil;
import org.littletonrobotics.junction.Logger;

public class Turret {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private Double desiredDeg = 0.0;
  private boolean minInclusive = false;
  private Timer hardwareTimer = new Timer();

  public enum turretState {
    DISABLED,
    SET_TURRET_ANGLE,
    UNWIND
  }

  public turretState state = turretState.DISABLED;

  public Turret(TurretIO io) {
    this.io = io;
    io.updateInputs(inputs);
    io.setPosition(getRotation()); // busted
    io.setPosition(0.5); // manual homing to the rear
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
            /*
            case DISABLED -> {
              break;
            }
            case UNWIND -> {
              if (desiredDeg >= Constants.Turret.maxUnwindLimitDeg) {
                desiredDeg = (desiredDeg - 360);
              } else if (desiredDeg <= Constants.Turret.minUnwindLimitDeg) {
                desiredDeg = (desiredDeg + 360);
              } else {
                desiredDeg = 0.0;
              }
              if (!needsToUnwind()
                  && (isAtGoal() || (inputs.turretDegs >= -180 && inputs.turretDegs <= 180))) {
                state = turretState.SET_TURRET_ANGLE;
              }
            }
            case SET_TURRET_ANGLE -> {
              if (needsToUnwind()) {
                state = turretState.UNWIND;
              }
              if (desiredDeg != null) {
                io.setAngle(desiredDeg);
              }
            } */
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
    double diffFromMid =
        Units.radiansToDegrees(
            MathUtil.angleModulus(
                Units.degreesToRadians(desiredDeg - Constants.Turret.midPointPhysicalDeg)));
    desiredDeg = Constants.Turret.midPointPhysicalDeg + diffFromMid;
    if (RobotContainer.intake.isExtended()) {
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
      if (!MathUtil.isNear(desiredDeg, inputs.turretDegs, Constants.Turret.goalToleranceDeg)) {
        hardwareTimer.start();
      } else {
        hardwareTimer.stop();
        hardwareTimer.reset();
      }

      if (hardwareTimer.hasElapsed(
          Shooter.isScoring()
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
