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
import frc.robot.subsystems.shooter.firingManager.FiringManager;
import frc.robot.subsystems.shooter.firingManager.FiringManager.FiringSolution;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.spindexer.Spindexer;
import frc.robot.subsystems.shooter.tunnel.Tunnel;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.vision.visionGlobalPose.VisionGlobalPose;
import frc.robot.util.GeomUtil;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  public enum ShooterState {
    DISABLED,
    IDLE, // Spindexer stopped, flywheel at full speed, tunnel at full speed
    UNWIND,
    PRESHOOT, // Flywheel gets up to speed; Turret/hood aim
    SHOOT, // Spindexer and tunnel get up to speed
    STOP // Everything but flywheel stopped
  }

  private ShooterState state = ShooterState.DISABLED;
  private ShooterState prevstate = ShooterState.DISABLED;

  private Flywheel flywheel;
  private Hood hood;
  private Spindexer spindexer;
  private Tunnel tunnel;
  private Turret turret;
  private Drive drive;
  private LED led;
  private boolean dontWantPremtiveUnwind;

  private double targetHoodAngleDeg;
  private double targetFlywheelSpeedRPS;
  private double targetTurretAngleDeg;
  private double targetTunnelSpeedRPS;
  private double targetSpindexerSpeedRPS;
  private boolean unwindComplete = true;
  private boolean isUnwinding = false;
  private boolean inIdle = true;
  private boolean fixedPositionShooting = false;
  private boolean isScoring = true;

  private boolean autoShootEnabled = false;

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
    flywheel.inputsPeriodic();
    hood.inputsPeriodic();
    tunnel.inputsPeriodic();
    spindexer.inputsPeriodic();
  }

  public void outputsPeriodic() {

    if (!Constants.turretLocked) {
      turret.inputsPeriodic();
    }
    calculateFiringSolution();

    if (Constants.firingManagerMode == Constants.SubsystemMode.TUNING) {

      flywheel.requestGoal(targetFlywheelSpeedRPS, null);
      hood.requestGoal(targetHoodAngleDeg, null);

      turret.requestAngle(targetTurretAngleDeg, null);

      tunnel.requestGoal(targetTunnelSpeedRPS);
      spindexer.requestGoal(targetSpindexerSpeedRPS);

      flywheel.outputsPeriodic();
      hood.outputsPeriodic();
      tunnel.outputsPeriodic();
      spindexer.outputsPeriodic();

      if (!Constants.turretLocked) {
        turret.outputsPeriodic();
      }
      return;
    }
    if (DriverStation.isDisabled()) {

      state = ShooterState.DISABLED;
    }

    if (turret.needsToUnwind()) {
      targetSpindexerSpeedRPS = 0;
    }

    if ((turret.needsToUnwind() && spindexer.isStopped()) || turret.isUnwinding()) {
      state = ShooterState.UNWIND;
      unwindComplete = false;
    }

    switch (state) {
      case DISABLED -> {
        if (DriverStation.isEnabled()) {

          state = ShooterState.IDLE;
        }
      }
      case IDLE -> {
        spindexer.requestIdle();
        turret.requestAngle(targetTurretAngleDeg, null);
        hood.requestGoal(Constants.Hood.safeAngleDeg, null);
        if (spindexer.isStopped()) {
          tunnel.requestIdle();
        } else {
          tunnel.requestGoal(targetTunnelSpeedRPS);
        }
        if (tunnel.isStopped()) {
          flywheel.requestGoal(Constants.Flywheel.idleRPS, null);
        }
      }
      case STOP -> {
        flywheel.requestGoal(0, null);
        hood.requestGoal(targetHoodAngleDeg, null);
        turret.requestAngle(targetTurretAngleDeg, null);
        spindexer.requestIdle();
        tunnel.requestIdle();
      }
      case UNWIND -> {
        spindexer.requestIdle();
        if (!tunnel.isStopped() && !spindexer.isStopped()) {
          turret.requestAngle(targetTurretAngleDeg, null);
        }
        hood.requestGoal(targetHoodAngleDeg, null);

        if (spindexer.isStopped()) {
          tunnel.requestIdle();
          if (tunnel.isStopped()) {
            turret.unwind(true);
            flywheel.requestGoal(Constants.Flywheel.idleRPS, null);
            targetFlywheelSpeedRPS = Constants.Flywheel.idleRPS;
          }
        }
        // In case for some reason we end up in this state when turret is locked
        if ((turret.isAtGoal() && turret.atTurretAtUnwindLimit()) || Constants.turretLocked) {
          unwindComplete = true;
          turret.unwind(false);
        }
      }
      case PRESHOOT -> {
        spindexer.requestIdle();
        flywheel.requestGoal(targetFlywheelSpeedRPS, isScoring);
        hood.requestGoal(targetHoodAngleDeg, isScoring);
        turret.requestAngle(targetTurretAngleDeg, isScoring);
      }
      case SHOOT -> {
        flywheel.requestGoal(targetFlywheelSpeedRPS, isScoring);
        hood.requestGoal(targetHoodAngleDeg, isScoring);
        turret.requestAngle(targetTurretAngleDeg, isScoring);
        tunnel.requestGoal(targetTunnelSpeedRPS);
        spindexer.requestGoal(targetSpindexerSpeedRPS);
      }
    }

    flywheel.outputsPeriodic();
    spindexer.outputsPeriodic();
    tunnel.outputsPeriodic();
    hood.outputsPeriodic();

    if (!Constants.turretLocked) {
      turret.outputsPeriodic();
    }

    Logger.recordOutput("Shooter/State", state.toString());
    Logger.recordOutput("Shooter/unwindComplete", unwindComplete);
    Logger.recordOutput("Shooter/spindexerStopped", spindexer.isStopped());
    Logger.recordOutput("Shooter/tunnelStopped", tunnel.isStopped());
    Logger.recordOutput("Shooter/flywheelAtSpeed", flywheel.isAtGoal());
    Logger.recordOutput("Shooter/hoodAtPosition", hood.isAtGoal());
    Logger.recordOutput("Shooter/turretAtPosition", turret.isAtGoal());
    Logger.recordOutput("Shooter/TargetHoodAngleDeg", targetHoodAngleDeg);
    Logger.recordOutput("Shooter/TargetFlywheelSpeedRPS", targetFlywheelSpeedRPS);
    Logger.recordOutput("Shooter/TargetTurretAngleDeg", targetTurretAngleDeg);
    Logger.recordOutput("Shooter/TargetTunnelSpeedRPS", targetTunnelSpeedRPS);
    Logger.recordOutput("Shooter/TargetIndexerSpeedRPS", targetSpindexerSpeedRPS);
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
      targetSpindexerSpeedRPS = Constants.fixedSolutionBlue.indexerSpeedRPS();
      if (Robot.alliance == DriverStation.Alliance.Blue) {
        targetTurretAngleDeg = Constants.fixedSolutionBlue.turretAngleDeg();
      } else {
        targetTurretAngleDeg = Constants.fixedSolutionRed.turretAngleDeg();
      }
    } else {
      FiringSolution firingSolution =
          FiringManager.getFiringSolution(drive.getTurretPose(), drive.getVelocity(), isScoring);
      targetHoodAngleDeg = firingSolution.hoodAngle();
      targetFlywheelSpeedRPS = firingSolution.flywheelSpeedRPS();
      targetTurretAngleDeg = firingSolution.turretAngleDeg();
      targetTunnelSpeedRPS = firingSolution.tunnelSpeedRPS();
      targetSpindexerSpeedRPS = firingSolution.indexerSpeedRPS();
    }
  }

  public ShooterState getState() {
    return state;
  }

  public void requestShoot(boolean fixedPosition, boolean isScoring) {
    prevstate = ShooterState.SHOOT;
    this.isScoring = isScoring;
    inIdle = false;
    fixedPositionShooting = fixedPosition;
    Logger.recordOutput("Shooter/currentMethod", "requestShoot()");
    if (Constants.firingManagerMode == Constants.SubsystemMode.TUNING) {
      return;
    }
    // Otherwise start shooting sequence
    if ((state == ShooterState.PRESHOOT
            || state == ShooterState.IDLE
            || state == ShooterState.STOP
            || (state == ShooterState.UNWIND))
        && unwindComplete) {
      if (state == ShooterState.PRESHOOT) {
        if (hood.isAtGoal() && flywheel.isAtGoal() && turret.isAtGoal()) {
          state = ShooterState.SHOOT;
        }
      } else {
        state = ShooterState.PRESHOOT;
        prevstate = ShooterState.PRESHOOT;
      }
    }
  }

  public void requestIdle() {
    prevstate = ShooterState.IDLE;
    inIdle = true;
    Logger.recordOutput("Shooter/currentMethod", "requestIdle()");
    if (!unwindComplete) {
      state = ShooterState.IDLE;
    }
  }

  public void requestStop() {
    prevstate = ShooterState.STOP;
    inIdle = true;
    Logger.recordOutput("Shooter/currentMethod", "requestStop()");
    if (!unwindComplete) {
      state = ShooterState.STOP;
    }
  }

  public boolean isInIdle() {
    return inIdle;
  }

  public void unjamOverride(boolean unjamOverride) {
    tunnel.unjamOverride(unjamOverride);
    spindexer.unjamOverride(unjamOverride);
  }

  public void trenchOverride(boolean unjamOverride) {
    hood.trenchOverride(unjamOverride);
  }

  public boolean isScoring() {
    return isScoring;
  }

  public boolean autoShootEnabled() {
    return autoShootEnabled;
  }
}
