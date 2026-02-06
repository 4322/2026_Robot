package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;

public class Turret {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private double desiredAzimuth = 0.0;
  private boolean safeToUnwind = true;

  public enum turretState {
    IDLE,
    SET_TURRET_ANGLE
  }

  public turretState state = turretState.IDLE;

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
          case IDLE -> {}
          case SET_TURRET_ANGLE -> {}
        }
      }
    }
  }

  public void setAngle(double angle, boolean safeToUnwind) {}

  public boolean isAtGoal() {
    return MathUtil.isNear(desiredAzimuth, inputs.turretDegs, Constants.Turret.tolerance);
  }

  public void preemptiveUnwind() {
    io.setAngle(Constants.Turret.zeroAzimuth);
  }

  public void setBrakeMode(Boolean mode) {}
}
