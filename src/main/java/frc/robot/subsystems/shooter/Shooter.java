package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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
import frc.robot.util.GeomUtil;
import frc.robot.util.HubShiftUtil;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  public enum ShooterState {
    DISABLED,
    IDLE, // Spindexer stopped, flywheel at full speed, tunnel at full speed
    UNWIND,
    PRESHOOT, // Flywheel gets up to speed; Turret/hood aim
    SHOOT, // Spindexer and tunnel get up to speed
    TRENCH,
    UNJAM,
    STOP // Everything but flywheel stopped
  }

  private ShooterState state = ShooterState.DISABLED;

  private Flywheel flywheel;
  private Hood hood;
  private Spindexer spindexer;
  private Tunnel tunnel;
  private Turret turret;
  private Drive drive;
  private LED led;

  private double targetHoodAngleDeg;
  private double targetFlywheelSpeedRPS;
  private double targetTurretAngleDeg;
  private double targetTunnelSpeedRPS;
  private double targetIndexerSpeedRPS;

  private boolean unwindComplete = false;
  private boolean inIdle = true;
  private boolean fixedPositionShooting = false;

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
    this.drive = drive;
  }

  public double getTargetTurretAngleDeg() {
    return targetTurretAngleDeg;
  }

  @Override
  public void periodic() {
    calculateFiringSolution();
    if (AreaManager.isHoodDangerZone(drive.getTurretPosition())) {
      state = ShooterState.TRENCH;
    }
    if (AreaManager.isTrench(drive.getTurretPosition())) {
      state = ShooterState.IDLE;
    }

    if (Constants.firingManagerMode == Constants.SubsystemMode.TUNING) {
      flywheel.requestGoal(targetFlywheelSpeedRPS);
      hood.requestGoal(targetHoodAngleDeg);

      turret.requestAngle(targetTurretAngleDeg, true);

      tunnel.requestGoal(targetTunnelSpeedRPS);
      spindexer.requestGoal(targetIndexerSpeedRPS);
      flywheel.periodic();
      spindexer.periodic();
      tunnel.periodic();
      hood.periodic();
      if (!Constants.turretLocked) {
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
      case TRENCH -> {
        hood.requestGoal(Constants.Hood.safeAngleDeg);
        spindexer.requestIdle();
        tunnel.requestIdle();
        flywheel.requestGoal(Constants.Flywheel.idleRPS);
        targetFlywheelSpeedRPS = Constants.Flywheel.idleRPS;
        if (!AreaManager.isTrench(drive.getTurretPosition())
            && !AreaManager.isHoodDangerZone(drive.getTurretPosition())) {
          state = ShooterState.IDLE;
        }
      }
      case IDLE -> {
        spindexer.requestIdle();
        turret.requestAngle(targetTurretAngleDeg, true);
        if (spindexer.isStopped()) {
          tunnel.requestIdle();
        } else {
          tunnel.requestGoal(targetTunnelSpeedRPS);
        }
        if (tunnel.isStopped()) {
          flywheel.requestGoal(Constants.Flywheel.motorId);
        }

        if (AreaManager.isTrench(drive.getTurretPosition())) {
          hood.requestGoal(Constants.Hood.safeAngleDeg);
        } else {
          Logger.recordOutput("Shooter/isHoodDangerZone", false);
          hood.requestGoal(targetHoodAngleDeg);
        }
      }
      case STOP -> {
        flywheel.requestGoal(Constants.Flywheel.idleRPS);
        hood.requestGoal(targetHoodAngleDeg);
        turret.requestAngle(targetTurretAngleDeg, true);
        spindexer.requestIdle();
        tunnel.requestIdle();
      }
      case UNWIND -> {
        spindexer.requestIdle();
        turret.requestAngle(targetTurretAngleDeg, false);
        hood.requestGoal(targetHoodAngleDeg);

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
        spindexer.requestIdle();
        flywheel.requestGoal(targetFlywheelSpeedRPS);
        hood.requestGoal(targetHoodAngleDeg);
        turret.requestAngle(targetTurretAngleDeg, true);
      }
      case SHOOT -> {
        flywheel.requestGoal(targetFlywheelSpeedRPS);
        hood.requestGoal(targetHoodAngleDeg);
        turret.requestAngle(targetTurretAngleDeg, false);
        tunnel.requestGoal(targetTunnelSpeedRPS);
        if (tunnel.getVelocity() > Constants.Tunnel.minPercentVelocity * targetTunnelSpeedRPS) {
          spindexer.requestGoal(targetIndexerSpeedRPS);
        } else {
          spindexer.requestIdle();
        }
      }
      case UNJAM -> {
        tunnel.requestGoal(Constants.Tunnel.unjamRPS);
        spindexer.requestGoal(Constants.Spindexer.unjamRPS);
      }
    }

    flywheel.periodic();
    spindexer.periodic();
    tunnel.periodic();
    hood.periodic();

    if (!Constants.turretLocked) {
      turret.periodic();
    }

    led.requestTurretUnwinding(state == ShooterState.UNWIND);

    Logger.recordOutput("Shooter/State", state.toString());
    Logger.recordOutput("Shooter/unwindComplete", unwindComplete);
    Logger.recordOutput("Shooter/spindexerStopped", spindexer.isStopped());
    Logger.recordOutput("Shooter/tunnelStopped", tunnel.isStopped());
    Logger.recordOutput(
        "Shooter/currentZone", AreaManager.getZoneOfPosition(drive.getTurretPosition()));
    Logger.recordOutput("Shooter/flywheelAtSpeed", flywheel.atTargetVelocity());
    Logger.recordOutput("Shooter/hoodAtPosition", hood.isAtGoal());
    Logger.recordOutput("Shooter/turretAtPosition", turret.isAtGoal());
    Logger.recordOutput("Shooter/TargetHoodAngleDeg", targetHoodAngleDeg);
    Logger.recordOutput("Shooter/TargetFlywheelSpeedRPS", targetFlywheelSpeedRPS);
    Logger.recordOutput("Shooter/TargetTurretAngleDeg", targetTurretAngleDeg);
    Logger.recordOutput("Shooter/TargetTunnelSpeedRPS", targetTunnelSpeedRPS);
    Logger.recordOutput("Shooter/TargetIndexerSpeedRPS", targetIndexerSpeedRPS);
    Logger.recordOutput("Shooter/CurrentTurretPose", drive.getTurretPose(turret.getAngle()));
    Logger.recordOutput(
        "Shooter/TargetTurretPose",
        GeomUtil.pose2dToPose3d(drive.getTurretPose(targetTurretAngleDeg), 0.4));
    Logger.recordOutput(
        "Shooter/ComponentPoses",
        new Pose3d[] {
          GeomUtil.pose2dToPose3d(
              new Pose2d(
                  Constants.Turret.originToTurret,
                  new Rotation2d(Units.degreesToRadians(targetTurretAngleDeg) - 4 * Math.PI / 3)),
              0.22)
        });
  }

  private void calculateFiringSolution() {
    if (fixedPositionShooting) {
      targetHoodAngleDeg = Constants.fixedSolutionBlue.hoodAngle();
      targetFlywheelSpeedRPS = Constants.fixedSolutionBlue.flywheelSpeedRPS();

      targetTunnelSpeedRPS = Constants.fixedSolutionBlue.tunnelSpeedRPS();
      targetIndexerSpeedRPS = Constants.fixedSolutionBlue.indexerSpeedRPS();
      if (Robot.alliance == DriverStation.Alliance.Blue) {
        targetTurretAngleDeg = Constants.fixedSolutionBlue.turretAngleDeg();
      } else {
        targetTurretAngleDeg = Constants.fixedSolutionRed.turretAngleDeg();
      }
    }

    FiringSolution firingSolution =
        FiringManager.getFiringSolution(
            drive.getTurretPose(),
            drive.getVelocity(),
            AreaManager.getZoneOfPosition(drive.getTurretPosition()) == Zone.ALLIANCE_ZONE);
    targetHoodAngleDeg = firingSolution.hoodAngle();
    targetFlywheelSpeedRPS = firingSolution.flywheelSpeedRPS();
    targetTurretAngleDeg = firingSolution.turretAngleDeg();
    targetTunnelSpeedRPS = firingSolution.tunnelSpeedRPS();
    targetIndexerSpeedRPS = firingSolution.indexerSpeedRPS();
  }

  public ShooterState getState() {
    return state;
  }

  public void requestShoot(boolean fixedPosition) {
    inIdle = false;
    fixedPositionShooting = fixedPosition;
    Logger.recordOutput("Shooter/currentMethod", "requestShoot()");
    if (Constants.firingManagerMode == Constants.SubsystemMode.TUNING) {
      return;
    }
    // If in alliance zone and shift not active
    if (AreaManager.getZoneOfPosition(drive.getRobotPose().getTranslation()) == Zone.ALLIANCE_ZONE
        && !HubShiftUtil.getShiftedShiftInfo().active()) {
      if (Constants.turretLocked) {
        state = ShooterState.STOP;
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
      // Otherwise start shooting sequence
      if (state == ShooterState.PRESHOOT
          || state == ShooterState.IDLE
          || state == ShooterState.UNJAM
          || state == ShooterState.STOP
          || (state == ShooterState.UNWIND && unwindComplete)) {
        unwindComplete = false;
        state = ShooterState.PRESHOOT;
        calculateFiringSolution(); // TODO optimize out
        if (hood.isAtGoal() && flywheel.atTargetVelocity() && turret.isAtGoal()) {
          state = ShooterState.SHOOT;
        }
      } else {
        // Seperate if to prevent warnings DO NOT COMBINE
        if (!Constants.turretLocked) {
          if (turret.needsToUnwind()) {
            unwindComplete = false;
            state = ShooterState.UNWIND;
          }
        }
      }
    }
  }

  public void requestIdle() {
    inIdle = true;
    Logger.recordOutput("Shooter/currentMethod", "requestIdle()");
    state = ShooterState.IDLE;
    // TODO deal with UNWIND state
  }

  public void requestStop() {
    inIdle = true;
    Logger.recordOutput("Shooter/currentMethod", "requestStop()");
    state = ShooterState.STOP;
    // TODO deal with UNWIND state
  }

  public void requestUnjam() {
    Logger.recordOutput("Shooter/currentMethod", "requestUnjam(");
    state = ShooterState.UNJAM;
    // TODO deal with UNWIND state
  }

  public boolean isInIdle() {
    return inIdle;
  }
}
