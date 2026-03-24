package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.subsystems.shooter.areaManager.AreaManager.Zone;
import frc.robot.subsystems.shooter.firingManager.FiringManager.FiringSolution;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.spindexer.Spindexer;
import frc.robot.subsystems.shooter.tunnel.Tunnel;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.GeomUtil;
import frc.robot.util.HubShiftUtil;
import org.littletonrobotics.junction.Logger;

public class BallPath extends SubsystemBase {

  public enum ShooterState {
    DISABLED,
    IDLE, // Spindexer stopped, flywheel at full speed, tunnel at full speed
    SHOOT, // Spindexer and tunnel get up to speed
    UNJAM,
  }

  private ShooterState ballState = ShooterState.DISABLED;

  private Flywheel flywheel;
  private Hood hood;
  private Spindexer spindexer;
  private Tunnel tunnel;
  private Turret turret;
  private Drive drive;
  private LED led;
  private boolean tunnelAlreadyStarted;

  private boolean fixedPositionShooting = false;

  private FiringSolution currentFiringSolution;

  public BallPath(Spindexer spindexer, Tunnel tunnel, LED led) {
    this.spindexer = spindexer;
    this.tunnel = tunnel;
    this.led = led;
  }

  @Override
  public void periodic() {

    if (Constants.firingManagerMode == Constants.SubsystemMode.TUNING) {
      tunnel.requestGoal(currentFiringSolution.tunnelSpeedRPS());
      spindexer.requestGoal(currentFiringSolution.indexerSpeedRPS());

      spindexer.periodic();
      tunnel.periodic();

      return;
    }
    if (DriverStation.isDisabled()) {

      ballState = ShooterState.DISABLED;
    }

    switch (ballState) {
      case DISABLED -> {}
      case IDLE -> {
        tunnelAlreadyStarted = false;
        spindexer.requestIdle();
        if (spindexer.isStopped()) {
          tunnel.requestIdle();
        } else {
          tunnel.requestGoal(currentFiringSolution.tunnelSpeedRPS());
        }
      }
      case SHOOT -> {
        // Tunnel and/or spindexer get up to speed
        tunnel.requestGoal(currentFiringSolution.tunnelSpeedRPS());
        if (tunnel.getVelocity()
            > Constants.Tunnel.minPercentVelocity * currentFiringSolution.tunnelSpeedRPS() || tunnelAlreadyStarted) {
          spindexer.requestGoal(currentFiringSolution.indexerSpeedRPS());
          tunnelAlreadyStarted = true;
        } else {
          spindexer.requestIdle();
        }
      }
      case UNJAM -> {
         tunnelAlreadyStarted = false;
        tunnel.requestGoal(Constants.Tunnel.unjamRPS);
        spindexer.requestGoal(Constants.Spindexer.unjamRPS);
      }
    }

    spindexer.periodic();
    tunnel.periodic();

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
                  new Rotation2d(
                      Units.degreesToRadians(currentFiringSolution.turretAngleDeg())
                          - 4 * Math.PI / 3)),
              0.22)
        });
  }

  public ShooterState getBallPathState() {
    return ballState;
  }

  public void setBallPathShoot() {
    ballState = ShooterState.SHOOT;
  }

  public void setBallPathIdle() {
    ballState = ShooterState.IDLE;
  }

  public void setBallPathUnjam() {
    ballState = ShooterState.UNJAM;
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

  public boolean restrictAllianceShoot() {
    return AreaManager.getZoneOfPosition(drive.getRobotPose().getTranslation())
            == Zone.ALLIANCE_ZONE
        && !HubShiftUtil.getShiftedShiftInfo().active();
  }

  public boolean getInAllianceZone() {
    return AreaManager.getZoneOfPosition(drive.getRobotPose().getTranslation())
        == Zone.ALLIANCE_ZONE;
  }

  public boolean isDriveInShootingArea() {
    return AreaManager.isShootingArea(drive.getRobotPose().getTranslation());
  }

  public boolean ballPathStopped() {
    return tunnel.isStopped() && spindexer.isStopped();
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
