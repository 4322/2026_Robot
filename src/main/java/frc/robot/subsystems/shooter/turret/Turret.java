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
  private boolean minInclusive = false;

  public enum turretState {
    DISABLED,
    SET_TURRET_ANGLE,
    UNWIND
  }

  public turretState state = turretState.DISABLED;

  public Turret(TurretIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/State", state);
    switch (Constants.turretMode) {
      case DISABLED -> {}
      case TUNING -> {}
      case NORMAL -> {
        switch (state) {
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
            if (desiredDeg == null) {
              io.setAngle(desiredDeg);
            } else {
              io.setPosition(getRotation());
            }
          }
        }
      }
    }
  }

  public void requestAngle(Double angle, boolean safeToUnwind) {
    this.desiredDeg = angle;
    if (Constants.turretLocked) {
      return;
    }
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
    Logger.recordOutput("Turret/desiredDeg", desiredDeg);
  }

  public boolean needsToUnwind() {
    return (inputs.turretDegs >= Constants.Turret.maxUnwindLimitDeg
        || inputs.turretDegs <= Constants.Turret.minUnwindLimitDeg
        || !RobotContainer.isDriveInShootingArea());
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
    }
    return MathUtil.isNear(desiredDeg, inputs.turretDegs, Constants.Turret.goalToleranceDeg);
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
    int CANCoderOneMod;
    int CANCoderTwoMod;
    int resolution = 4096;
    int CANcoderOneRotations = 0;
    int CANcoderTwoRotations = 0;
    int prevRotationsOne = 0;
    int prevRotationsTwo = 0;
    double turretMod;
    if (inputs.encoderOneRotations < prevRotationsOne - resolution / 2) {
      CANcoderOneRotations++;
    } else if (inputs.encoderOneRotations > prevRotationsOne + resolution / 2) {
      CANcoderOneRotations--;
    }
    if (inputs.encoderTwoRotations < prevRotationsTwo - resolution / 2) {
      CANcoderTwoRotations++;
    } else if (inputs.encoderTwoRotations > prevRotationsTwo + resolution / 2) {
      CANcoderTwoRotations--;
    }

    prevRotationsOne = (int) inputs.encoderOneRotations;
    prevRotationsTwo = (int) inputs.encoderTwoRotations;

    CANCoderOneMod = mod((int) CANcoderOneRotations, (int) Constants.Turret.CANCoderOneRatio);
    CANCoderTwoMod = mod((int) CANcoderTwoRotations, (int) Constants.Turret.CANCoderTwoRatio);

    turretMod = mod((10 * CANCoderOneMod) + (36 * CANCoderTwoMod), 45);

    return turretMod
        + (inputs.encoderOneRotations / (double) resolution / Constants.Turret.CANCoderTwoRatio);
  }

  private int mod(int a, int b) {
    return ((a % b) + b) % b;
  }
}
