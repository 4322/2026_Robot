package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.FiringParameters;
import frc.robot.constants.Constants.FiringTarget;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.subsystems.shooter.areaManager.AreaManager.Zone;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.spindexer.Spindexer;
import frc.robot.subsystems.shooter.tunnel.Tunnel;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.vision.visionGlobalPose.VisionGlobalPose;
import frc.robot.util.GeomUtil;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.firecontrol.ProjectileSimulator;
import frc.robot.util.firecontrol.ShotCalculator;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  public enum ShooterState {
    DISABLED,
    IDLE,
    UNWIND,
    PRESHOOT, // Flywheel gets up to speed; Turret/hood aim
    SHOOT, // Spindexer and tunnel get up to speed
    TRENCH,
    UNJAM
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

  private FiringTarget firingTarget;
  private ShotCalculator.ShotInputs shotCalculatorInputs;
  private ShotCalculator.LaunchParameters launchParameters;
  private ShotCalculator shotCalculator;
  private ProjectileSimulator.GeneratedLUT lut;
  private FiringParameters firingParameters;

  private boolean unwindComplete = false;
private boolean inIdle = true;
  private ProjectileSimulator sim;

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

    ShotCalculator.Config config = new ShotCalculator.Config();
    config.launcherOffsetX =
        Constants.Turret.originToTurret
            .getX(); // how far forward the launcher is from robot center (m)
    config.launcherOffsetY = Constants.Turret.originToTurret.getY(); // how far left, 0 if centered
    config.phaseDelayMs = Constants.VisionGlobalPose.phaseDelayMs; // your vision pipeline latency
    config.mechLatencyMs =
        Constants.Shooter.mechLatencyMs; // how long the mechanism takes to respond
    config.maxTiltDeg =
        Constants.Shooter.maxTiltDeg; // suppress firing when chassis tilts past this (bumps/ramps)
    config.headingSpeedScalar =
        Constants.Shooter
            .headingSpeedScalar; // heading tolerance tightens with robot speed (0 to disable)
    config.headingReferenceDistance =
        Constants.Shooter
            .headingReferenceDistance; // heading tolerance scales with distance from hub

    this.shotCalculator = new ShotCalculator();

    ProjectileSimulator sim = new ProjectileSimulator(Constants.ShotCalculator.params);
    this.lut = sim.generateLUT();

    for (var entry : lut.entries()) {
      if (entry.reachable()) {
        System.out.printf(
            "%.2fm -> %.0f RPM, %.3fs TOF%n", entry.distanceM(), entry.rpm(), entry.tof());
      }
    }
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
        if (AreaManager.isTrench(drive.getTurretPosition())) {
          hood.requestGoal(Constants.Hood.safeAngleDeg);
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
        flywheel.requestGoal(targetFlywheelSpeedRPS);
        hood.requestGoal(targetHoodAngleDeg);
        turret.requestAngle(targetTurretAngleDeg, false);
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
        GeomUtil.pose2dToPose3d(drive.getTurretPose(targetTurretAngleDeg), 0.6));
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
    firingTarget = getShootingTarget(drive.getRobotPose().getTranslation());
    shotCalculatorInputs =
        new ShotCalculator.ShotInputs(
            drive.getRobotPose(),
            drive.getFieldVelocity(),
            drive.getRobotVelocity(),
            firingTarget.translation(),
            firingTarget.forward(),
            0.9, // vision confidence, 0 to 1
            0, // pitch for tilt gate (0.0 if no gyro)
            0 // roll for tilt gate (0.0 if no gyro)
            );
    launchParameters = shotCalculator.calculate(shotCalculatorInputs);
    if (launchParameters.isValid() && launchParameters.confidence() > 50) {
      if (Constants.ShotCalculator.useSimulatedShotTuning) {
        targetFlywheelSpeedRPS = launchParameters.rpm() / 60;
        targetTurretAngleDeg = launchParameters.driveAngle().getDegrees();
        targetHoodAngleDeg = Constants.ShotCalculator.hoodAngle;
        targetTunnelSpeedRPS = Constants.Tunnel.shootRPS;
        targetIndexerSpeedRPS = Constants.Spindexer.shootRPS;
      } else {
        if (AreaManager.getZoneOfPosition(drive.getTurretPose().getTranslation())
            == Zone.ALLIANCE_ZONE) {
          firingParameters =
              Constants.FiringManager.firingMapScoring.get(launchParameters.solvedDistanceM());

        } else {
          firingParameters =
              Constants.FiringManager.firingMapPassing.get(launchParameters.solvedDistanceM());
        }
        targetFlywheelSpeedRPS = firingParameters.getFlywheelRPM() / 60;
        targetHoodAngleDeg = firingParameters.getHoodAngleDeg();
        targetTunnelSpeedRPS = firingParameters.getTunnelRPS();
        targetIndexerSpeedRPS = firingParameters.getIndexerRPS();
        targetTurretAngleDeg = launchParameters.driveAngle().getDegrees();
      }
    }
  }

  public ShooterState getState() {
    return state;
  }

  public void requestShoot() {
    inIdle = false;
    Logger.recordOutput("Shooter/currentMethod", "requestShoot()");
    if (Constants.firingManagerMode == Constants.SubsystemMode.TUNING) {
      return;
    }
    // If in alliance zone and shift not active
    if (AreaManager.getZoneOfPosition(drive.getRobotPose().getTranslation()) == Zone.ALLIANCE_ZONE
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
      // Otherwise start shooting sequence
      if (state == ShooterState.PRESHOOT
          || state == ShooterState.IDLE
          || state == ShooterState.UNJAM
          || (state == ShooterState.UNWIND && unwindComplete)) {
        unwindComplete = false;
        state = ShooterState.PRESHOOT;
        calculateFiringSolution();
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
    if (state == ShooterState.UNWIND && unwindComplete) {
      state = ShooterState.IDLE;
    } else if (state == ShooterState.PRESHOOT || state == ShooterState.SHOOT) {
      unwindComplete = false;
      state = ShooterState.UNWIND;
    }
  }

  public void requestUnjam() {
    Logger.recordOutput("Shooter/currentMethod", "requestUnjam(");
    state = ShooterState.UNJAM;
  }

  public void endIdle() {
    inIdle = false;
  }

  public boolean isInIdle() {
    return inIdle;
  }

  private FiringTarget getShootingTarget(Translation2d robotPosition) {
    Zone zone = AreaManager.getZoneOfPosition(robotPosition);

    if (Robot.alliance == Alliance.Blue) {

      switch (zone) {
        case ALLIANCE_ZONE:
          Logger.recordOutput("FiringManager/targetZone", "Hub");
          return Constants.FiringTargetTranslations.Blue.hubTarget;
        case RIGHT_NEUTRAL:
          Logger.recordOutput("FiringManager/targetZone", "Alliance Right");
          return Constants.FiringTargetTranslations.Blue.allianceRightTarget;
        case LEFT_NEUTRAL:
          Logger.recordOutput("FiringManager/targetZone", "Alliance Left");
          return Constants.FiringTargetTranslations.Blue.allianceLeftTarget;
        case RIGHT_OPPOSITION:
          if (Constants.FiringManager.alwaysTargetAllianceZone) {
            Logger.recordOutput("FiringManager/targetZone", "Alliance Right");
            return Constants.FiringTargetTranslations.Blue.allianceRightTarget;
          } else {
            Logger.recordOutput("FiringManager/targetZone", "Neutral Right");
            return Constants.FiringTargetTranslations.Blue.neutralRightTarget;
          }
        case LEFT_OPPOSITION:
          if (Constants.FiringManager.alwaysTargetAllianceZone) {
            Logger.recordOutput("FiringManager/targetZone", "Alliance Left");
            return Constants.FiringTargetTranslations.Blue.allianceLeftTarget;
          } else {
            Logger.recordOutput("FiringManager/targetZone", "Neutral Left");
            return Constants.FiringTargetTranslations.Blue.neutralLeftTarget;
          }
        default:
          Logger.recordOutput("FiringManager/targetZone", "Invalid");
          return new FiringTarget(new Translation2d(), new Translation2d());
      }
    } else {
      switch (zone) {
        case ALLIANCE_ZONE:
          Logger.recordOutput("FiringManager/targetZone", "Hub");
          return Constants.FiringTargetTranslations.Red.hubTarget;
        case RIGHT_NEUTRAL:
          Logger.recordOutput("FiringManager/targetZone", "Alliance Right");
          return Constants.FiringTargetTranslations.Red.allianceRightTarget;
        case LEFT_NEUTRAL:
          Logger.recordOutput("FiringManager/targetZone", "Alliance Left");
          return Constants.FiringTargetTranslations.Red.allianceLeftTarget;
        case RIGHT_OPPOSITION:
          if (Constants.FiringManager.alwaysTargetAllianceZone) {
            Logger.recordOutput("FiringManager/targetZone", "Alliance Right");
            return Constants.FiringTargetTranslations.Red.allianceRightTarget;
          } else {
            Logger.recordOutput("FiringManager/targetZone", "Neutral Right");
            return Constants.FiringTargetTranslations.Red.neutralRightTarget;
          }
        case LEFT_OPPOSITION:
          if (Constants.FiringManager.alwaysTargetAllianceZone) {
            Logger.recordOutput("FiringManager/targetZone", "Alliance Left");
            return Constants.FiringTargetTranslations.Red.allianceLeftTarget;
          } else {
            Logger.recordOutput("FiringManager/targetZone", "Neutral Left");
            return Constants.FiringTargetTranslations.Red.neutralLeftTarget;
          }
        default:
          Logger.recordOutput("FiringManager/targetZone", "Invalid");
          return new FiringTarget(new Translation2d(), new Translation2d());
      }
    }
  }

  public boolean shouldSimShoot() {
    return spindexer.getVelocity() > 0.5 * targetIndexerSpeedRPS;
  }

  public Translation3d getShotVelocity() {
    if (launchParameters != null) {
      return new Translation3d(); // TODO
    } else {
      return new Translation3d();
    }
  }
}
