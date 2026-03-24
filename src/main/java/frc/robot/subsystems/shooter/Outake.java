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

public class Outake extends SubsystemBase {

  public enum ShooterState {
    DISABLED,
    IDLE, // Spindexer stopped, flywheel at full speed, tunnel at full speed
    SHOOT // Spindexer and tunnel get up to speed
  }

  public static enum ShootGoal{
    PASSING,
    SCORING
  }

   private ShooterState outakeState = ShooterState.DISABLED;
    private static ShootGoal shooting = ShootGoal.SCORING;

  private Flywheel flywheel;
  private Hood hood;
  private Spindexer spindexer;
  private Tunnel tunnel;
  private Turret turret;
  private Drive drive;
  private LED led;


  private boolean fixedPositionShooting = false;

  private FiringSolution currentFiringSolution;

  public Outake(
      Flywheel flywheel,
      Hood hood,
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
    if (AreaManager.getZoneOfPosition(drive.getRobotPose().getTranslation()) == Zone.ALLIANCE_ZONE){
      shooting = ShootGoal.SCORING;
    } else{
      shooting = ShootGoal.PASSING;
    }
    if (Constants.firingManagerMode == Constants.SubsystemMode.TUNING) {
      flywheel.requestGoal(currentFiringSolution.flywheelSpeedRPS());
      hood.requestGoal(currentFiringSolution.hoodAngle());

      turret.requestAngle(currentFiringSolution.turretAngleDeg(), true);
      flywheel.periodic();

      hood.periodic();
      if (!Constants.turretLocked) {
        turret.periodic();
      }
      return;
    }
    if (DriverStation.isDisabled()) {

      outakeState = ShooterState.DISABLED;
    }
    switch (outakeState){
      case DISABLED ->{}
      case IDLE -> {
      hood.requestGoal(Constants.Hood.safeAngleDeg);
      tunnel.requestIdle();
      flywheel.requestGoal(Constants.Flywheel.idleRPS);
      }
      case SHOOT ->{
      turret.requestAngle(currentFiringSolution.turretAngleDeg(), false);
      flywheel.requestGoal(currentFiringSolution.flywheelSpeedRPS());
      hood.requestGoal(currentFiringSolution.hoodAngle());
      } 
    
    }
    flywheel.periodic();
    hood.periodic();
    if (!Constants.turretLocked) {
      turret.periodic();
    }



  
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

  public ShooterState getOutakeState() {
    return outakeState;
  }



  public void setOutakeShoot() {
    outakeState = ShooterState.SHOOT;
  }

  

  public void setOutakeIdle() {
    if (ballPathStopped()){
    outakeState = ShooterState.IDLE;
    }
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

  public boolean restrictAllianceShoot(){
  return AreaManager.getZoneOfPosition(drive.getRobotPose().getTranslation()) == Zone.ALLIANCE_ZONE
        && !HubShiftUtil.getShiftedShiftInfo().active();
  }
 public boolean isDriveInShootingArea() {
    return AreaManager.isShootingArea(drive.getRobotPose().getTranslation());
  }
  public boolean ballPathStopped(){
    return tunnel.isStopped() && spindexer.isStopped();
  }

  public boolean isFlywheelAtSpeed() {
    return flywheel.atTargetVelocity();
  }

  public boolean isHoodAtPosition() {
    return hood.isAtGoal();
  }

  public static boolean isScoring(){
    return shooting == ShootGoal.SCORING;
  }

  public boolean isTurretAtPosition() {
    return turret.isAtGoal();
  }

   public boolean isTurretNeedToUnwind() {
    return turret.needsToUnwind();
  }
} 
