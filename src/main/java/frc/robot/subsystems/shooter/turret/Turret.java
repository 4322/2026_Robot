package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;
import frc.robot.util.ClockUtil;
import org.littletonrobotics.junction.Logger;

public class Turret {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private Double desiredDeg = 0.0;
  private double lastDesiredDeg = 0.0;
  private boolean safeToUnwind = false;
  private boolean minInclusive = false;
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
          case SET_TURRET_ANGLE -> {
            if (desiredDeg != null && !safeToUnwind) {
              lastDesiredDeg = desiredDeg;
              io.setAngle(desiredDeg);
            } else if (safeToUnwind) {
              desiredDeg =
                  MathUtil.isNear(Constants.Turret.midPointPhysicalDeg, desiredDeg, 180)
                      ? desiredDeg
                      : Constants.Turret.midPointPhysicalDeg;
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
    if (desiredDeg != null) {
      if (inputs.turretDegs + 180 >= Constants.Turret.maxPhysicalLimitDeg) {
        minInclusive = true;
        ;
      } else if (inputs.turretDegs - 180 <= Constants.Turret.minPhysicalLimitDeg) {
        minInclusive = true;
      }
      desiredDeg =
          ClockUtil.inputModulus(angle - inputs.turretDegs, -180, 180, minInclusive)
              + inputs.turretDegs;
      if (desiredDeg >= Constants.Turret.maxUnwindLimitDeg) {
        desiredDeg = Constants.Turret.maxPhysicalLimitDeg;
      } else if (desiredDeg <= Constants.Turret.minUnwindLimitDeg) {
        desiredDeg = Constants.Turret.minPhysicalLimitDeg;
      }
    } else if (desiredDeg == null) {
      safeToUnwind = true;
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
  }

  public void setBrakeMode(Boolean mode) {
    io.setBrakeMode(mode);
  }
}
