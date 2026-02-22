package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
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
  private LED led;

  private double targetHoodAngleDeg;
  private double targetFlywheelSpeedRPM;
  private double targetTurretAngleDeg;

  private boolean unwindComplete = false;

  public Shooter(
      Flywheel flywheel,
      Hood hood,
      Spindexer spindexer,
      Tunnel tunnel,
      Turret turret,
      VisionGlobalPose visionGlobalPose,
      Drive drive,
      LED led) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.spindexer = spindexer;
    this.tunnel = tunnel;
    this.turret = turret;
    this.led = led;
    this.visionGlobalPose = visionGlobalPose;
    this.drive = drive;
  }

  @Override
  public void periodic() {
    calculateFiringSolution();
    switch (state) {
      case DISABLED -> {
        if (DriverStation.isEnabled()) {

          state = ShooterState.IDLE;
        }
      }
      case IDLE -> {
        spindexer.requestIdle();
        turret.setAngle(targetTurretAngleDeg, true);

        if (AreaManager.isHoodDangerZone(drive.getPose().getTranslation())) {
          hood.requestGoal(Constants.Hood.idleAngleDeg);
        } else {
          hood.requestGoal(targetHoodAngleDeg);
        }

        if (spindexer.isStopped()) {
          tunnel.requestIdle();
          if (tunnel.isStopped()) {
            flywheel.requestGoal(Constants.Flywheel.idleRPS);
          }
        }
      }
      case UNWIND -> {
        spindexer.requestIdle();
        turret.setAngle(targetTurretAngleDeg, false);

        if (AreaManager.isHoodDangerZone(drive.getPose().getTranslation())) {
          hood.requestGoal(Constants.Hood.idleAngleDeg);
        } else {
          hood.requestGoal(targetHoodAngleDeg);
        }

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
        flywheel.requestGoal(targetFlywheelSpeedRPM / 60);
        hood.requestGoal(targetHoodAngleDeg);
        turret.setAngle(targetTurretAngleDeg, false);

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

    led.requestTurretUnwinding(state == ShooterState.UNWIND);

    Logger.recordOutput("Shooter/State", state.toString());
  }

  private void calculateFiringSolution() {
    FiringSolution firingSolution =
        FiringManager.getFiringSolution(drive.getPose().getTranslation(), drive.getVelocity());
    targetHoodAngleDeg = firingSolution.hoodAngle();
    targetFlywheelSpeedRPM = firingSolution.flywheelSpeedRPM();
    targetTurretAngleDeg = firingSolution.turretAngleDeg();
  }

  public ShooterState getState() {
    return state;
  }

  public void requestShoot() {

    if ((AreaManager.getZoneOfPosition(drive.getPose().getTranslation()) == Zone.ALLIANCE_ZONE
            && !HubTracker.isAbleToShoot())
        || (AreaManager.isTrenchNoShootingArea(drive.getPose().getTranslation()))) {
      state = ShooterState.IDLE;

    } else {
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
    }
  }

  public void requestIdle() {
    if (state == ShooterState.UNWIND && unwindComplete) {
      state = ShooterState.IDLE;
    } else if (state == ShooterState.PRESHOOT || state == ShooterState.SHOOT) {
      unwindComplete = false;
      state = ShooterState.UNWIND;
    }
  }
}
