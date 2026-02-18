package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;
import frc.robot.util.ClockUtil;
import org.littletonrobotics.junction.Logger;

public class Turret {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private Double desiredDeg = 0.0;
  private boolean safeToUnwind = false;
  private boolean minInclusive = false;
  private double turretAzimuth = 0.0;

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
    Logger.recordOutput("Turret/State", state.toString());
    switch (Constants.turretMode) {
      case DISABLED -> {}
      case TUNING -> {}
      case DRIVE_TUNING -> {}
      case NORMAL -> {
        switch (state) {
          case DISABLED -> {
            break;
          }
          case UNWIND -> {
            if (desiredDeg != null && !isAtGoal()) {
              io.setAngle(desiredDeg);
            } else if (desiredDeg != null && isAtGoal()) {
              safeToUnwind = false;
            }
          }
          case SET_TURRET_ANGLE -> {
            if (desiredDeg != null) {
              io.setAngle(desiredDeg);
            }
          }
        }
      }
    }
  }

  public void setAngle(Double angle, boolean safeToUnwind) {
    this.desiredDeg = angle;
    this.safeToUnwind = safeToUnwind;
    if (desiredDeg != null && state != turretState.UNWIND) {
      if (inputs.turretDegs + 180 >= Constants.Turret.maxPhysicalLimitDeg) {
        minInclusive = true;
        ;
      } else if (inputs.turretDegs - 180 <= Constants.Turret.minPhysicalLimitDeg) {
        minInclusive = true;
      }
      desiredDeg = angleDistance(desiredDeg, inputs.turretDegs, minInclusive);
    } else if (desiredDeg == null || this.safeToUnwind && (state != turretState.UNWIND)) {
      state = turretState.UNWIND;
      if (desiredDeg == null) {
        desiredDeg = Constants.Turret.midPointPhysicalDeg;
      } else {
        setAngleWithinMidpoint();
      }
      safeToUnwind = true;
    } else if (state == turretState.UNWIND && desiredDeg != null) {
      setAngleWithinMidpoint();
      state = !safeToUnwind ? turretState.SET_TURRET_ANGLE : state;
    }
    if (desiredDeg != null) {
      if (desiredDeg >= Constants.Turret.maxUnwindLimitDeg) {
        desiredDeg = Constants.Turret.maxPhysicalLimitDeg;
      } else if (desiredDeg <= Constants.Turret.minUnwindLimitDeg) {
        desiredDeg = Constants.Turret.minPhysicalLimitDeg;
      }
    }
    this.turretAzimuth = inputs.turretDegs % 360;
    Logger.recordOutput("Turret/turretAzimuth", turretAzimuth);
  }

  public boolean needsToUnwind() {
    return (inputs.turretDegs >= Constants.Turret.maxUnwindLimitDeg
        || inputs.turretDegs <= Constants.Turret.minUnwindLimitDeg);
  }

  public boolean isAtGoal() {
    return MathUtil.isNear(desiredDeg, inputs.turretDegs, Constants.Turret.goalToleranceDeg);
  }

  public void setTurretAngleState() {
    state = turretState.SET_TURRET_ANGLE;
  }

  public void unwind() {
    safeToUnwind = true;
    state = turretState.UNWIND;
    desiredDeg = Constants.Turret.midPointPhysicalDeg;
  }

  public void setBrakeMode(Boolean mode) {
    io.setBrakeMode(mode);
  }

  public double angleDistance(double targetAngle, double currentAngle, boolean minInclusive) {
    double angleDistance =
        ClockUtil.inputModulus(targetAngle - currentAngle, -180, 180, minInclusive) + currentAngle;
    return angleDistance;
  }

  private double setAngleWithinMidpoint() {
    if (inputs.turretDegs > Constants.Turret.midPointPhysicalDeg) {
      desiredDeg = angleDistance(desiredDeg, Constants.Turret.maxMidPointPhysicalDeg, false);
    } else if (inputs.turretDegs < Constants.Turret.midPointPhysicalDeg) {
      desiredDeg = angleDistance(desiredDeg, Constants.Turret.minMidPointPhysicalDeg, true);
    } else {
      desiredDeg = Constants.Turret.midPointPhysicalDeg;
    }
    return desiredDeg =
        MathUtil.isNear(Constants.Turret.midPointPhysicalDeg, desiredDeg, 180)
            ? desiredDeg
            : Constants.Turret.midPointPhysicalDeg;
  }
}
