package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;

public class Turret {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private Double desiredAzimuth = 0.0;
  private boolean safeToUnwind = true;

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
            // TODO
          }
        }
      }
    }
  }

  public void setAngle(Double angle, boolean safeToUnwind) {
    this.desiredAzimuth = angle;
    // Rewinds when angle is null, and is safe to unwind
    // Goes to side it favors, and if curr angle + desi big than max, set to max, but when safe
    // unwind

  }

  public boolean safeToUnwind() {
    if (this.desiredAzimuth == null
        || (inputs.turretDegs >= Constants.Turret.maxRequestAzimuth
            || inputs.turretDegs <= Constants.Turret.minRequestAzimuth)) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isAtGoal() {
    return MathUtil.isNear(desiredAzimuth, inputs.turretDegs, Constants.Turret.tolerance);
  }

  public void setTurretAzimuth() {
    state = turretState.SET_TURRET_ANGLE;
  }

  public void preemptiveUnwind() {
    desiredAzimuth = Constants.Turret.offsetAzimuth;
  }

  public void setBrakeMode(Boolean mode) {}
}
