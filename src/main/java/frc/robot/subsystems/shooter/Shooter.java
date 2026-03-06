package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.util.HubShiftUtil;
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
  private double targetFlywheelSpeedRPS;
  private double targetTurretAngleDeg;
  private double targetTunnelSpeedRPS;
  private double targetIndexerSpeedRPS;

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

    if (Constants.firingManager == Constants.SubsystemMode.TUNING) {
      flywheel.requestGoal(targetFlywheelSpeedRPS);
      hood.requestGoal(targetHoodAngleDeg);
      if (!Constants.turretLocked){
        turret.setAngle(targetTurretAngleDeg, true);
      }
      tunnel.requestIndex(targetTunnelSpeedRPS);
      spindexer.requestIndex(targetIndexerSpeedRPS);
      flywheel.periodic();
      spindexer.periodic();
      tunnel.periodic();
      hood.periodic();
      if (!Constants.turretLocked){
      turret.periodic();
      }
      return;
    }
    if (DriverStation.isDisabled()) {

      state = ShooterState.DISABLED;
    }

    switch (state) {
      case DISABLED -> {
        if (DriverStation.isEnabled()) {

          state = ShooterState.IDLE;
        }
      }
      case IDLE -> {
        spindexer.requestIdle();
        if (!Constants.turretLocked){
        turret.setAngle(targetTurretAngleDeg, true);
        }
        if (AreaManager.isHoodDangerZone(drive.getPose().getTranslation())) {
          Logger.recordOutput("Shooter/isHoodDangerZone", true);
          hood.requestGoal(Constants.Hood.idleAngleDeg);
          targetHoodAngleDeg = Constants.Hood.idleAngleDeg;
        } else {
          Logger.recordOutput("Shooter/isHoodDangerZone", false);
          hood.requestGoal(targetHoodAngleDeg);
        }

        if (spindexer.isStopped()) {
          tunnel.requestIdle();
          if (tunnel.isStopped()) {
            flywheel.requestGoal(Constants.Flywheel.idleRPS);
            targetFlywheelSpeedRPS = Constants.Flywheel.idleRPS;
          }
        }
      }
      case UNWIND -> {
        spindexer.requestIdle();
        turret.setAngle(targetTurretAngleDeg, false);

        if (AreaManager.isHoodDangerZone(drive.getPose().getTranslation())) {
          hood.requestGoal(Constants.Hood.idleAngleDeg);
          targetHoodAngleDeg = Constants.Hood.idleAngleDeg;
        } else {
          hood.requestGoal(targetHoodAngleDeg);
        }

        if (spindexer.isStopped()) {
          tunnel.requestIdle();
          if (tunnel.isStopped()) {
            turret.unwind();
            flywheel.requestGoal(Constants.Flywheel.idleRPS);
            targetFlywheelSpeedRPS = Constants.Flywheel.idleRPS;
          }
        }
        // In case for some reason we end up in this state when turret is locked
        if (turret.isAtGoal() || Constants.turretLocked) {
          unwindComplete = true;
        }
      }
      case PRESHOOT -> {
        flywheel.requestGoal(targetFlywheelSpeedRPS);
        hood.requestGoal(targetHoodAngleDeg);
        if (!Constants.turretLocked){
        turret.setAngle(targetTurretAngleDeg, true);
        } else {
          drive.alignToTarget();
        }
      }
      case SHOOT -> {
        flywheel.requestGoal(targetFlywheelSpeedRPS);
        hood.requestGoal(targetHoodAngleDeg);
        if (!Constants.turretLocked){
        turret.setAngle(targetTurretAngleDeg, false);
        } else {
          drive.alignToTarget();
        }

        tunnel.requestIndex(
            Constants.Tunnel.dynamicVelocity
                ? (flywheel.getVelocity() / (targetFlywheelSpeedRPS)) * targetTunnelSpeedRPS
                : targetTunnelSpeedRPS);
        if (tunnel.getVelocity() > Constants.Tunnel.minPercentVelocity * targetTunnelSpeedRPS) {
          spindexer.requestIndex(
              Constants.Spindexer.dynamicVelocity
                  ? (tunnel.getVelocity() / targetTunnelSpeedRPS) * targetIndexerSpeedRPS
                  : targetIndexerSpeedRPS);
        }
      }
    }

    flywheel.periodic();
    spindexer.periodic();
    tunnel.periodic();
    hood.periodic();
    if (!Constants.turretLocked){
    turret.periodic();
    }

    led.requestTurretUnwinding(state == ShooterState.UNWIND);

    Logger.recordOutput("Shooter/State", state.toString());
    Logger.recordOutput("Shooter/unwindComplete", unwindComplete);
    Logger.recordOutput("Shooter/spindexerStopped", spindexer.isStopped());
    Logger.recordOutput("Shooter/tunnelStopped", tunnel.isStopped());
    Logger.recordOutput(
        "Shooter/currentZone", AreaManager.getZoneOfPosition(drive.getPose().getTranslation()));
    Logger.recordOutput("Shooter/flywheelAtSpeed", flywheel.atTargetVelocity());
    Logger.recordOutput("Shooter/hoodAtPosition", hood.isAtGoal());
    Logger.recordOutput("Shooter/turretAtPosition", turret.isAtGoal());
    Logger.recordOutput("Shooter/TargetHoodAngleDeg", targetHoodAngleDeg);
    Logger.recordOutput("Shooter/TargetFlywheelSpeedRPS", targetFlywheelSpeedRPS / 60);
    Logger.recordOutput("Shooter/TargetTurretAngleDeg", targetTurretAngleDeg);
    Logger.recordOutput(
        "Shooter/TurretPose",
        new Pose3d[] {
          new Pose3d(
              -0.13, 0.17, 0.3, new Rotation3d(0, 0, Units.degreesToRadians(turret.getAngle())))
        });
  }

  private void calculateFiringSolution() {
    FiringSolution firingSolution =
        FiringManager.getFiringSolution(
            drive.getPose().getTranslation(),
            drive.getVelocity(),
            AreaManager.getZoneOfPosition(drive.getPose().getTranslation()) == Zone.ALLIANCE_ZONE);
    targetHoodAngleDeg = firingSolution.hoodAngle();
    targetFlywheelSpeedRPS = firingSolution.flywheelSpeedRPS();
    targetTurretAngleDeg = firingSolution.turretAngleDeg();
    targetTunnelSpeedRPS = firingSolution.tunnelSpeedRPS();
    targetIndexerSpeedRPS = firingSolution.indexerSpeedRPS();
  }

  public ShooterState getState() {
    return state;
  }

  public void requestShoot() {
    Logger.recordOutput("Shooter/currentMethod", "requestShoot()");
    if (Constants.firingManager == Constants.SubsystemMode.TUNING) {
      return;
    }

    if ((AreaManager.isTrench(drive.getPose().getTranslation()))) {
      // Emergency hood lower
      state = ShooterState.IDLE;

    } else if (AreaManager.getZoneOfPosition(drive.getPose().getTranslation()) == Zone.ALLIANCE_ZONE
        && !HubShiftUtil.getShiftedShiftInfo().active()) {
      if (Constants.turretLocked) {
        state = ShooterState.IDLE;
        return;
      }
      // Don't shoot if inactive
      if (turret.needsToUnwind()) {
        unwindComplete = false;
        state = ShooterState.UNWIND;
      }
      if (state == ShooterState.UNWIND && unwindComplete) {
        unwindComplete = false;
        state = ShooterState.IDLE;
      }

    } else {
      if (state == ShooterState.PRESHOOT
          || state == ShooterState.IDLE
          || (state == ShooterState.UNWIND && unwindComplete)) {
        unwindComplete = false;
        state = ShooterState.PRESHOOT;
        calculateFiringSolution();
        if (hood.isAtGoal() && flywheel.atTargetVelocity() && ((turret.isAtGoal() && !Constants.turretLocked) || (Constants.turretLocked && drive.atRotationTarget()))) {
          state = ShooterState.SHOOT;
        }
      } else {
        if (!Constants.turretLocked && turret.needsToUnwind()) {
          unwindComplete = false;
          state = ShooterState.UNWIND;
        }
      }
    }
  }

  public void requestIdle() {
    Logger.recordOutput("Shooter/currentMethod", "requestIdle()");
    if (state == ShooterState.UNWIND && unwindComplete) {
      state = ShooterState.IDLE;
    } else if (state == ShooterState.PRESHOOT || state == ShooterState.SHOOT) {
      unwindComplete = false;
      state = ShooterState.UNWIND;
    }
  }
}
