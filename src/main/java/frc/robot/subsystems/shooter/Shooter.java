package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.subsystems.shooter.areaManager.AreaManager.Zone;
import frc.robot.subsystems.shooter.firingManager.FiringManager;
import frc.robot.subsystems.shooter.firingManager.FiringManager.FiringSolution;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.spindexer.Spindexer;
import frc.robot.subsystems.shooter.tunnel.Tunnel;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.vision.visionGlobalPose.VisionGlobalPose;
import frc.robot.util.HubTracker;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  public enum ShooterState {
    DISABLED,
    IDLE,
    UNWIND,
    PRESHOOT, // Flywheel gets up to speed; Turret/hood aim
    SHOOT, // Spindexer and tunnel get up to speed
  }

  private ShooterState state = ShooterState.DISABLED;

  private Flywheel flywheel;
  private Hood hood;
  private Spindexer spindexer;
  private Tunnel tunnel;
  private Turret turret;
  private VisionGlobalPose visionGlobalPose;
  private Drive drive;

  private double targetHoodAngleDeg;
  private double targetFlywheelSpeedRPM;
  private double targetTurretAngleDeg;

  private boolean unwindComplete = false;

  private boolean idleIsUnwinding = false;

  public Shooter(
      Flywheel flywheel,
      Hood hood,
      Spindexer spindexer,
      Tunnel tunnel,
      Turret turret,
      VisionGlobalPose visionGlobalPose,
      Drive drive) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.spindexer = spindexer;
    this.tunnel = tunnel;
    this.turret = turret;
    this.visionGlobalPose = visionGlobalPose;
    this.drive = drive;
  }

  @Override
  public void periodic() {
    switch (state) {
      case DISABLED -> {
        if (DriverStation.isEnabled()) {

          state = ShooterState.IDLE;
        }
      }
      case IDLE -> {
        spindexer.requestIdle();
        if (spindexer.isStopped()) {
          tunnel.requestIdle();
          flywheel.requestGoal(Constants.Flywheel.idleRPS);
          turret.setAngle(targetTurretAngleDeg, true);
          if (AreaManager.getZoneOfPosition(visionGlobalPose.getHybridPose(drive).getTranslation())
              == Zone.TRENCH_EXCLUSION) { // TODO set trench exclusion zone
            hood.requestGoal(Constants.Hood.idleAngleDeg);
          } else {
            hood.requestGoal(targetHoodAngleDeg);
          }
        }
      }
      case UNWIND -> {
        spindexer.requestIdle();
        if (spindexer.isStopped()) {
          tunnel.requestIdle();
          if (tunnel.isStopped()) {
            turret.unwind();
          }
        }
        if (turret.isAtGoal()) {
          unwindComplete = true;
        }
      }
      case PRESHOOT -> {
        flywheel.requestGoal(targetFlywheelSpeedRPM / 60);
        hood.requestGoal(targetHoodAngleDeg);
        turret.setAngle(targetTurretAngleDeg, true);
      }
      case SHOOT -> {
        calculateFiringSolution();
        flywheel.requestGoal(targetFlywheelSpeedRPM / 60);
        hood.requestGoal(targetHoodAngleDeg);
        turret.setAngle(targetTurretAngleDeg, true);

        tunnel.requestIndex(
            Constants.Tunnel.dynamicVelocity
                ? Constants.Tunnel.dynamicVelocityPercent * flywheel.getVelocity()
                : Constants.Tunnel.indexingMechanismRotationsPerSec);
        spindexer.requestIndex(
            Constants.Spindexer.dynamicVelocity
                ? Constants.Spindexer.dynamicVelocityPercent * tunnel.getVelocity()
                : Constants.Spindexer.indexingMechanismRotationsPerSec);
      }
    }

    flywheel.periodic();
    spindexer.periodic();
    tunnel.periodic();
    hood.periodic();
    turret.periodic();

    Logger.recordOutput("Shooter/State", state.toString());
  }

  private void calculateFiringSolution() {
    FiringSolution firingSolution =
        FiringManager.getFiringSolution(
            visionGlobalPose.getHybridPose(drive).getTranslation(), drive.getVelocity());
    targetHoodAngleDeg = firingSolution.hoodAngle();
    targetFlywheelSpeedRPM = firingSolution.flywheelSpeedRPM();
    targetTurretAngleDeg = firingSolution.turretAngleDeg();
  }

  public boolean isUnwindComplete() {
    return unwindComplete;
  }

  // Turret is at maximum path of travel
  public boolean needsToUnwind() {
    return turret.needsToUnwind();
  }

  public boolean isMechanismsAtSpeed() {
    return flywheel.atTargetVelocity() && tunnel.isAtSpeed();
  }

  public boolean isFlywheelAtSpeed() {
    return flywheel.atTargetVelocity();
  }

  public boolean isHoodAtAngle() {
    return hood.isAtGoal();
  }

  public boolean isTurretInPosition() {
    return turret.isAtGoal();
  }

  public void setState(ShooterState newState) {
    if (!(newState == state)) {
      state = newState;
      unwindComplete = false;
    }
  }

  public ShooterState getState() {
    return state;
  }

  public void requestShoot() {
    calculateFiringSolution();

    if (!(AreaManager.getZoneOfPosition(drive.getPose().getTranslation()) == Zone.ALLIANCE_ZONE
        && !HubTracker.isAbleToShoot())) {
      if (state == ShooterState.IDLE || (state == ShooterState.UNWIND && unwindComplete)) {
        unwindComplete = false;
        state = ShooterState.PRESHOOT;
        if (hood.isAtGoal() && turret.isAtGoal()) {
          state = ShooterState.SHOOT;
        }
      } else {
        if (turret.needsToUnwind()) {
          unwindComplete = false;
          state = ShooterState.UNWIND;
        }
      }
    } else {
      state = ShooterState.IDLE;
    }
  }

  public void requestIdle(boolean forceUnwind) {
    if (forceUnwind) {
      turret.unwind();
      if (turret.isAtGoal()) {
        state = ShooterState.IDLE;
      } else {
        state = ShooterState.UNWIND;
      }
    } else {
      state = ShooterState.IDLE;
    }
  }
}
