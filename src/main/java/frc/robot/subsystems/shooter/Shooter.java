package frc.robot.subsystems.shooter;

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
  public void periodic() {}

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
        if (hood.isAtGoal() && turret.isAtGoal() && flywheel.atTargetVelocity()) {
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
    Logger.recordOutput("Shooter/currentMethod", "requestIdle()");
    if (state == ShooterState.UNWIND && unwindComplete) {
      state = ShooterState.IDLE;
    } else if (state == ShooterState.PRESHOOT || state == ShooterState.SHOOT) {
      unwindComplete = false;
      state = ShooterState.UNWIND;
    }
  }
}
