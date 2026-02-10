package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;

public class Turret {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private Double desiredDeg = 0.0;
  private boolean safeToUnwind = false;

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
    this.desiredDeg = angle;
    this.safeToUnwind = safeToUnwind;
    // Rewinds when angle is null, and is safe to unwind
    // Goes to side it favors, and if curr angle + desi big than max, set to max, but when safe
    // unwind
    if (MathUtil.isNear(angle, inputs.turretDegs, 180)) {}
  }

  public boolean needsToUnwind() {
    return (inputs.turretDegs >= Constants.Turret.maxUnwindLimitDeg
        || inputs.turretDegs <= Constants.Turret.minPhysicalLimitDeg);
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

  public void setBrakeMode(Boolean mode) {}
}
