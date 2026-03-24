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
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class BallPath extends SubsystemBase {

  public enum ShooterState {
    DISABLED,
    IDLE, // Spindexer stopped, flywheel at full speed, tunnel at full speed
    SHOOT, // Spindexer and tunnel get up to speed
    UNJAM,
    UNWIND
  }

  private ShooterState ballState = ShooterState.DISABLED;

  private Flywheel flywheel;
  private Hood hood;
  private Spindexer spindexer;
  private Tunnel tunnel;
  private Turret turret;
  private Drive drive;
  private LED led;

  private boolean fixedPositionShooting = false;

  private FiringSolution firingSolution;

  public BallPath(Spindexer spindexer, Tunnel tunnel, FiringSolution firingSolution, LED led) {
    this.spindexer = spindexer;
    this.tunnel = tunnel;
    this.firingSolution = firingSolution;

    this.led = led;
  }

  @Override
  public void periodic() {

    if (Constants.firingManagerMode == Constants.SubsystemMode.TUNING) {
      tunnel.requestGoal(firingSolution.tunnelSpeedRPS());
      spindexer.requestGoal(firingSolution.indexerSpeedRPS());

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
        spindexer.requestIdle();
        if (spindexer.isStopped()) {
          tunnel.requestIdle();
        } else {
          tunnel.requestGoal(firingSolution.tunnelSpeedRPS());
        }
      }
      case SHOOT -> {
        // Tunnel and/or spindexer get up to speed
        tunnel.requestGoal(firingSolution.tunnelSpeedRPS());
        spindexer.requestGoal(firingSolution.indexerSpeedRPS());
      }
      case UNJAM -> {
        tunnel.requestGoal(Constants.Tunnel.unjamRPS);
        spindexer.requestGoal(Constants.Spindexer.unjamRPS);
      }
      case UNWIND -> {
        spindexer.requestIdle();
        tunnel.requestIdle();
      }
    }

    spindexer.periodic();
    tunnel.periodic();

    Logger.recordOutput("Shooter/spindexerStopped", spindexer.isStopped());
    Logger.recordOutput("Shooter/tunnelStopped", tunnel.isStopped());
    Logger.recordOutput("Shooter/flywheelAtSpeed", flywheel.atTargetVelocity());
    Logger.recordOutput("Shooter/hoodAtPosition", hood.isAtGoal());
    Logger.recordOutput("Shooter/turretAtPosition", turret.isAtGoal());
    Logger.recordOutput("Shooter/TargetHoodAngleDeg", firingSolution.hoodAngle());
    Logger.recordOutput("Shooter/TargetFlywheelSpeedRPS", firingSolution.flywheelSpeedRPS());
    Logger.recordOutput("Shooter/TargetTurretAngleDeg", firingSolution.turretAngleDeg());
    Logger.recordOutput("Shooter/TargetTunnelSpeedRPS", firingSolution.tunnelSpeedRPS());
    Logger.recordOutput("Shooter/TargetIndexerSpeedRPS", firingSolution.indexerSpeedRPS());
    Logger.recordOutput("Shooter/CurrentTurretPose", drive.getTurretPose(turret.getAngle()));
    Logger.recordOutput(
        "Shooter/TargetTurretPose",
        GeomUtil.pose2dToPose3d(drive.getTurretPose(firingSolution.turretAngleDeg()), 0.4));
    Logger.recordOutput(
        "Shooter/ComponentPoses",
        new Pose3d[] {
          GeomUtil.pose2dToPose3d(
              new Pose2d(
                  Constants.Turret.originToTurret,
                  new Rotation2d(
                      Units.degreesToRadians(firingSolution.turretAngleDeg()) - 4 * Math.PI / 3)),
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

  public void setBallPathUnwind() {
    ballState = ShooterState.UNWIND;
  }

  public BooleanSupplier getBallIdle() {
    return () -> ballState == ShooterState.IDLE;
  }

  public void setFiringSolution(FiringSolution firingSolution) {
    this.firingSolution = firingSolution;
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
