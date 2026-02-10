package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;

public class Turret {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private Double desiredDeg = 0.0;
  private double lastDesiredDeg = 0.0;
  private boolean safeToUnwind = false;
  private double turretAzimuth = 0.0;

  public enum turretState {
    DISABLED,
    SET_TURRET_ANGLE
  }

  public turretState state = turretState.DISABLED;

  public Turret(TurretIO io) {
    this.io = io;
  }

  public void periodic() {
    switch (Constants.turretMode) {
      case DISABLED -> {}
      case TUNING -> {}
      case DRIVE_TUNING -> {}
      case NORMAL -> {
        switch (state) {
          case DISABLED -> {
            break;
          }
          case SET_TURRET_ANGLE -> {
            if (desiredDeg != null) {
              lastDesiredDeg = desiredDeg;
              io.setAngle(desiredDeg);
            } else if (desiredDeg == null && safeToUnwind) {
              io.setAngle(Constants.Turret.midPointPhysicalDeg);
            } else {
              io.setAngle(lastDesiredDeg);
            }
          }
        }
      }
    }
  }

  public void setAngle(Double angle, boolean safeToUnwind) {
    this.desiredDeg = angle;
    this.safeToUnwind = safeToUnwind;
    // Rewinds when angle is null, and is safe to unwind
    // Goes to side it favors, and if curr angle + desi big than max, set to max, but when safe
    // unwind
    if (desiredDeg != null) {
      this.turretAzimuth = angle;
      io.setAzimuth(turretAzimuth);
      if (inputs.turretDegs >= Constants.Turret.midPointPhysicalDeg) {
        if (angle < inputs.turretDegs - 180) {
          desiredDeg = angle + 360;
        } else {
          desiredDeg = angle;
        }
      } else {
        if (angle > inputs.turretDegs + 180) {
          desiredDeg = angle - 360;
        } else {
          desiredDeg = angle;
        }
      }
      if (desiredDeg >= Constants.Turret.maxPhysicalLimitDeg) {
        desiredDeg = Constants.Turret.maxPhysicalLimitDeg;
      } else if (desiredDeg <= Constants.Turret.minPhysicalLimitDeg) {
        desiredDeg = Constants.Turret.minPhysicalLimitDeg;
      }
    } else if (desiredDeg == null && safeToUnwind) {
      desiredDeg = Constants.Turret.midPointPhysicalDeg;
    }
  }

  public boolean needsToUnwind() {
    return (inputs.turretDegs >= Constants.Turret.maxUnwindLimitDeg
        || inputs.turretDegs <= Constants.Turret.minUnwindLimitDeg);
  }

  public boolean isAtGoal() {
    return MathUtil.isNear(desiredDeg, inputs.turretDegs, Constants.Turret.goalToleranceDeg);
  }

  public void setTurretDeg(Double deg) {
    state = turretState.SET_TURRET_ANGLE;
  }

  public void preemptiveUnwind() {
    desiredDeg = Constants.Turret.midPointPhysicalDeg;
  }

  public void setBrakeMode(Boolean mode) {
    io.setBrakeMode(mode);
  }
}
