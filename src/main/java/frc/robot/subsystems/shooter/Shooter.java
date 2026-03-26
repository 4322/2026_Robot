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
    STARTING_CONFIG,
    DISABLED,
    IDLE, // Spindexer stopped, flywheel at full speed, tunnel at full speed
    UNWIND,
    PRESHOOT, // Flywheel gets up to speed; Turret/hood aim
    SHOOT, // Spindexer and tunnel get up to speed
    STOP // Everything but flywheel stopped
  }

  private ShooterState state = ShooterState.STARTING_CONFIG;
  private ShooterState requestedState = ShooterState.STARTING_CONFIG;

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
  private double targetSpindexerSpeedRPS;
  private boolean fixedPositionShooting = false;
  private boolean isScoring = true;

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
    if (!Constants.turretLocked) {
      turret.inputsPeriodic();
    }
  }

  public void outputsPeriodic() {
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
    if (DriverStation.isDisabled() && state != ShooterState.STARTING_CONFIG) {
      state = ShooterState.DISABLED;
    }

    // Turret unwind logic blocks all other incoming state requests from commands
    // Once unwind is complete, other state requests are allowed and in cases where
    // no state request is made, goes back to previous state
    if (turret.needsToUnwind() || turret.isUnwinding()) {
      state = ShooterState.UNWIND;
    }

    switch (state) {
      case STARTING_CONFIG -> {
        // Intake has to be down first before using shooter
        // Commands will take state out of starting config only if intake was first deployed
      }
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
        // Keep requesting target states while waiting for unwind
        hood.requestGoal(targetHoodAngleDeg, null);
        flywheel.requestGoal(targetFlywheelSpeedRPS, null);
        // Keep sending turret angle requests and unwind logic will continuously adjust target
        // setpoint within range of physical midpoint
        turret.requestAngle(targetTurretAngleDeg, null);
        spindexer.requestIdle();

        if (spindexer.isStopped()) {
          // Since requested turret angle is constantly updating while we're unwinding
          // turret will attempt to go to requested setpoint within certain range of physical midpoint
          turret.unwind(true);
        }

        // TODO: Change check to when turret is within 90 degrees from physical midpoint
        // Don't need to check if turret is exactly at goal, just within range
        if ((turret.isAtGoal() && turret.requestAtUnwindLimit()) || Constants.turretLocked) {
          turret.unwind(false);
          // Exit unwind state when completed
          // In next loop cycle if no command sends shooter request,
          // shooter will go to last requested state before unwind started
          state = requestedState;
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

  // Needs to be continuously called in order to start shooting balls
  public void requestShoot(boolean fixedPosition, boolean isScoring) {
    this.isScoring = isScoring;
    this.fixedPositionShooting = fixedPosition;
    Logger.recordOutput("Shooter/currentMethod", "requestShoot()");
    if (Constants.firingManagerMode == Constants.SubsystemMode.TUNING) {
      return;
    }

    if (state != ShooterState.SHOOT && state != ShooterState.STARTING_CONFIG && state != ShooterState.DISABLED) {
      if (state == ShooterState.PRESHOOT) {
        if (hood.isAtGoal() && flywheel.isAtGoal() && turret.isAtGoal()) {
          state = ShooterState.SHOOT;
        }
      }
      else {
        // Never set requested state to SHOOT to ensure after turret unwinds
        // we wait for everything to get to setpoint before starting to shoot again
        state = ShooterState.PRESHOOT;
        requestedState = ShooterState.PRESHOOT;
      }
    }
  }

  public void requestIdle() {
    requestedState = ShooterState.IDLE;
    state = ShooterState.IDLE;
    Logger.recordOutput("Shooter/currentMethod", "requestIdle()");
  }

  public void requestStop() {
    requestedState = ShooterState.STOP;
    state = ShooterState.STOP;
    Logger.recordOutput("Shooter/currentMethod", "requestStop()");
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
}
