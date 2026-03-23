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

  private boolean fixedPositionShooting = false;

  private FiringSolution currentFiringSolution;

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
    this.currentFiringSolution = null;
  }

  @Override
  public void periodic() {

    if (Constants.firingManagerMode == Constants.SubsystemMode.TUNING) {
      flywheel.requestGoal(currentFiringSolution.flywheelSpeedRPS());
      hood.requestGoal(currentFiringSolution.hoodAngle());

      turret.requestAngle(currentFiringSolution.turretAngleDeg(), true);

      tunnel.requestGoal(currentFiringSolution.tunnelSpeedRPS());
      spindexer.requestGoal(currentFiringSolution.indexerSpeedRPS());
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
      case DISABLED -> {}
      case TRENCH -> {
        // Immediately force subsystems to safe positions and stop everything
        hood.requestGoal(Constants.Hood.safeAngleDeg);
        spindexer.requestIdle();
        tunnel.requestIdle();
        flywheel.requestGoal(Constants.Flywheel.idleRPS);
      }
      case IDLE -> {
        // Subsystems track target; flywheel at idle speed
        spindexer.requestIdle();
        turret.requestAngle(currentFiringSolution.turretAngleDeg(), true);
        if (spindexer.isStopped()) {
          tunnel.requestIdle();
        } else {
          tunnel.requestGoal(currentFiringSolution.tunnelSpeedRPS());
        }
        if (tunnel.isStopped()) {
          flywheel.requestGoal(Constants.Flywheel.idleRPS);
        }
      }
      case STOP -> {
        // Subsystems track target; everything else stopped
        flywheel.requestGoal(0);
        hood.requestGoal(currentFiringSolution.hoodAngle());
        turret.requestAngle(currentFiringSolution.turretAngleDeg(), true);
        spindexer.requestIdle();
        tunnel.requestIdle();
      }
      case PRESHOOT -> {
        // Subsystems track target
        spindexer.requestIdle();
        tunnel.requestIdle();
        flywheel.requestGoal(currentFiringSolution.flywheelSpeedRPS());
        hood.requestGoal(currentFiringSolution.hoodAngle());
        turret.requestAngle(currentFiringSolution.turretAngleDeg(), true);
      }
      case SHOOT -> {
        // Tunnel and/or spindexer get up to speed
        flywheel.requestGoal(currentFiringSolution.flywheelSpeedRPS());
        hood.requestGoal(currentFiringSolution.hoodAngle());
        turret.requestAngle(currentFiringSolution.turretAngleDeg(), false);
        tunnel.requestGoal(currentFiringSolution.tunnelSpeedRPS());
        if (tunnel.getVelocity() > Constants.Tunnel.minPercentVelocity * currentFiringSolution.tunnelSpeedRPS()) {
          spindexer.requestGoal(currentFiringSolution.indexerSpeedRPS());
        } else {
          spindexer.requestIdle();
        }
      }
      case UNJAM -> { // TODO figure out best way to unjam
        // Tunnel/spindexer in reverse
        flywheel.requestGoal(currentFiringSolution.flywheelSpeedRPS());
        hood.requestGoal(currentFiringSolution.hoodAngle());
        turret.requestAngle(currentFiringSolution.turretAngleDeg(), false);
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

  
    Logger.recordOutput("Shooter/spindexerStopped", spindexer.isStopped());
    Logger.recordOutput("Shooter/tunnelStopped", tunnel.isStopped());
    Logger.recordOutput("Shooter/flywheelAtSpeed", flywheel.atTargetVelocity());
    Logger.recordOutput("Shooter/hoodAtPosition", hood.isAtGoal());
    Logger.recordOutput("Shooter/turretAtPosition", turret.isAtGoal());
    Logger.recordOutput("Shooter/TargetHoodAngleDeg", currentFiringSolution.hoodAngle());
    Logger.recordOutput("Shooter/TargetFlywheelSpeedRPS", currentFiringSolution.flywheelSpeedRPS());
    Logger.recordOutput("Shooter/TargetTurretAngleDeg", currentFiringSolution.turretAngleDeg());
    Logger.recordOutput("Shooter/TargetTunnelSpeedRPS", currentFiringSolution.tunnelSpeedRPS());
    Logger.recordOutput("Shooter/TargetIndexerSpeedRPS", currentFiringSolution.indexerSpeedRPS());
    Logger.recordOutput("Shooter/CurrentTurretPose", drive.getTurretPose(turret.getAngle()));
    Logger.recordOutput(
        "Shooter/TargetTurretPose",
        GeomUtil.pose2dToPose3d(drive.getTurretPose(currentFiringSolution.turretAngleDeg()), 0.4));
    Logger.recordOutput(
        "Shooter/ComponentPoses",
        new Pose3d[] {
          GeomUtil.pose2dToPose3d(
              new Pose2d(
                  Constants.Turret.originToTurret,
                  new Rotation2d(Units.degreesToRadians(currentFiringSolution.turretAngleDeg()) - 4 * Math.PI / 3)),
              0.22)
        });
  }

  public ShooterState getState() {
    return state;
  }

  public void setState(ShooterState newState) {
    if (state == newState) {
      return;
    }
    state = newState;
  }

  public void setFiringSolution(FiringSolution firingSolution) {
    this.currentFiringSolution = firingSolution;
  }

  public boolean isSpindexerStopped() {
    return spindexer.isStopped();
  }

  public boolean isTunnelStopped() {
    return tunnel.isStopped();
  }

  public boolean isFlywheelAtSpeed() {
    return flywheel.atTargetVelocity();
  }

  public boolean isHoodAtPosition() {
    return hood.isAtGoal();
  }

  public boolean isTurretAtPosition() {
    return turret.isAtGoal();
  }
}
