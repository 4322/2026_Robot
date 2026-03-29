package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ShotCalculatorParameters;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.firingManager.FiringManager;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.spindexer.Spindexer;
import frc.robot.subsystems.shooter.tunnel.Tunnel;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.vision.visionGlobalPose.VisionGlobalPose;
import frc.robot.util.firecontrol.ShotCalculator;
import frc.robot.util.firecontrol.ShotLUT;
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
  private boolean doUnwind = false;
  private boolean fixedPositionShooting = false;
  private boolean isScoring = true;
  private Timer idleTimer = new Timer();
  private boolean resetIdleTimeout = false;

  private ShotCalculator shotCalc;

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
    config.phaseDelayMs = Constants.ShotCalculator.phaseDelayMs; // your vision pipeline latency
    config.mechLatencyMs =
        Constants.ShotCalculator.mechLatencyMs; // how long the mechanism takes to respond
    config.maxTiltDeg =
        Constants.ShotCalculator
            .maxTiltDeg; // suppress firing when chassis tilts past this (bumps/ramps)
    config.headingSpeedScalar =
        Constants.ShotCalculator
            .headingSpeedScalar; // heading tolerance tightens with robot speed (0 to disable)
    config.headingReferenceDistance =
        Constants.ShotCalculator
            .headingReferenceDistance; // heading tolerance scales with distance from hub

    shotCalc = new ShotCalculator(config);

    ShotLUT lut = new ShotLUT();
    for (ShotCalculatorParameters params : Constants.FiringManager.firingParametersListScoring) {
      lut.put(
          params.distanceMeters(),
          params.flywheelRPS() * 60.0,
          params.hoodAngleDeg(),
          params.timeOfFlightSec());
    }
    shotCalc.loadShotLUT(lut);
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
    SmartDashboard.putNumber("Shooter/TargetFlywheelSpeedRPS", targetFlywheelSpeedRPS);

    if (Constants.firingManagerMode == Constants.SubsystemMode.TUNING) {

      flywheel.requestGoal(targetFlywheelSpeedRPS, false);
      hood.requestGoal(targetHoodAngleDeg, false);

      turret.requestAngle(targetTurretAngleDeg, false);

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
      // Reset variable in case disabling during unwind
      doUnwind = false;
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
        resetIdleTimeout = true;
        if (DriverStation.isEnabled()) {
          state = ShooterState.IDLE;
        }
      }
      case IDLE -> {
        if (resetIdleTimeout) {
          idleTimer.restart();
          resetIdleTimeout = false;
        }

        spindexer.requestIdle();
        turret.requestAngle(targetTurretAngleDeg, isScoring);

        if (spindexer.isStopped()) {
          tunnel.requestIdle();
        }

        if (idleTimer.hasElapsed(Constants.Flywheel.idleTimeout)) {
          flywheel.requestGoal(Constants.Flywheel.idleRPS, isScoring);
        } else {
          flywheel.requestGoal(targetFlywheelSpeedRPS, isScoring);
        }

        if (idleTimer.hasElapsed(Constants.Hood.idleTimeout)) {
          hood.requestGoal(Constants.Hood.safeAngleDeg, isScoring);
        } else {
          hood.requestGoal(targetHoodAngleDeg, isScoring);
        }
      }
      case STOP -> {
        resetIdleTimeout = true;
        flywheel.requestGoal(0, isScoring);
        hood.requestGoal(targetHoodAngleDeg, isScoring);
        turret.requestAngle(targetTurretAngleDeg, isScoring);
        spindexer.requestIdle();
        tunnel.requestIdle();
      }
      case UNWIND -> {
        // Optimization: Keep requesting target states while waiting for unwind if shooting
        if (requestedState == ShooterState.PRESHOOT) {
          hood.requestGoal(targetHoodAngleDeg, isScoring);
          flywheel.requestGoal(targetFlywheelSpeedRPS, isScoring);
        }

        // Keep sending turret angle requests and unwind logic will continuously adjust target
        // setpoint within range of physical midpoint
        turret.requestAngle(targetTurretAngleDeg, isScoring);
        spindexer.requestIdle();

        if (spindexer.isStopped()) {
          // Since requested turret angle is constantly updating while we're unwinding
          // turret will attempt to go to requested setpoint within certain range of physical
          // midpoint
          doUnwind = true;
        }

        if (turret.atUnwindLimit() || Constants.turretLocked) {
          doUnwind = false;
          // Exit unwind state when completed
          // In next loop cycle if no command sends shooter request,
          // shooter will go to last requested state before unwind started
          state = requestedState;
        }
        turret.unwind(doUnwind);
      }
      case PRESHOOT -> {
        resetIdleTimeout = true;
        spindexer.requestIdle();
        flywheel.requestGoal(targetFlywheelSpeedRPS, isScoring);
        hood.requestGoal(targetHoodAngleDeg, isScoring);
        turret.requestAngle(targetTurretAngleDeg, isScoring);
      }
      case SHOOT -> {
        resetIdleTimeout = true;
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
    Logger.recordOutput("Shooter/TargetHoodAngleDeg", targetHoodAngleDeg);
    Logger.recordOutput("Shooter/TargetFlywheelSpeedRPS", targetFlywheelSpeedRPS);
    Logger.recordOutput("Shooter/TargetTurretAngleDeg", targetTurretAngleDeg);
    Logger.recordOutput("Shooter/TargetTunnelSpeedRPS", targetTunnelSpeedRPS);
    Logger.recordOutput("Shooter/TargetIndexerSpeedRPS", targetSpindexerSpeedRPS);
  }

  private void calculateFiringSolution() {
    if (fixedPositionShooting) {
      targetHoodAngleDeg = Constants.fixedSolutionBlue.hoodAngle;
      targetFlywheelSpeedRPS = Constants.fixedSolutionBlue.flywheelSpeedRPS;
      targetTunnelSpeedRPS = Constants.fixedSolutionBlue.tunnelSpeedRPS;
      targetSpindexerSpeedRPS = Constants.fixedSolutionBlue.indexerSpeedRPS;
      if (Robot.alliance == DriverStation.Alliance.Blue) {
        targetTurretAngleDeg = Constants.fixedSolutionBlue.turretAngleDeg;
      } else {
        targetTurretAngleDeg = Constants.fixedSolutionRed.turretAngleDeg;
      }
    } else {

      if (isScoring) {
        Translation2d hubCenter;
        Translation2d hubForward;

        if (Robot.alliance == DriverStation.Alliance.Red) {
          hubCenter = FieldConstants.Red.hubTranslation;
          hubForward = new Translation2d(-1, 0);
          hubCenter.plus(new Translation2d(Units.inchesToMeters(-3), 0));
        } else {
          hubCenter = FieldConstants.Blue.hubTranslation;
          hubForward = new Translation2d(1, 0);
          hubCenter.plus(new Translation2d(Units.inchesToMeters(3), 0));
        }

        ShotCalculator.ShotInputs inputs =
            new ShotCalculator.ShotInputs(
                drive.getRobotPose(),
                drive.getFieldRelativeVelocity(),
                drive.getRobotRelativeVelocity(),
                hubCenter,
                hubForward,
                0.9, // vision confidence, 0 to 1
                0, // pitch for tilt gate (0.0 if no gyro)
                0 // roll for tilt gate (0.0 if no gyro)
                );

        ShotCalculator.LaunchParameters shot = shotCalc.calculate(inputs);
        if (shot.isValid()) {
          targetHoodAngleDeg = shotCalc.getHoodAngle(shot.solvedDistanceM());
          targetFlywheelSpeedRPS = shot.rpm() / 60.0;
          targetTurretAngleDeg = shot.turretAngle().getDegrees();
          targetTunnelSpeedRPS = Constants.Tunnel.shootRPS;
          targetSpindexerSpeedRPS = Constants.Spindexer.shootRPS;
          return;
        }

      } else {
        FiringSolution firingSolution =
            FiringManager.getFiringSolution(drive.getTurretPose(), drive.getVelocity(), isScoring);

        targetHoodAngleDeg = firingSolution.hoodAngle;
        targetFlywheelSpeedRPS = firingSolution.flywheelSpeedRPS;
        targetTurretAngleDeg = firingSolution.turretAngleDeg;
        targetTunnelSpeedRPS = firingSolution.tunnelSpeedRPS;
        targetSpindexerSpeedRPS = firingSolution.indexerSpeedRPS;
      }
    }
  }

  public ShooterState getState() {
    return state;
  }

  public boolean hoodAtGoal() {
    return hood.isAtGoal();
  }

  // Needs to be continuously called in order to start shooting balls
  public void requestShoot(boolean fixedPosition, boolean isScoring) {
    this.isScoring = isScoring;
    this.fixedPositionShooting = fixedPosition;
    Logger.recordOutput("Shooter/currentMethod", "requestShoot()");
    if (Constants.firingManagerMode == Constants.SubsystemMode.TUNING) {
      return;
    }

    if (state != ShooterState.SHOOT) {
      if (state == ShooterState.PRESHOOT) {
        if (hood.isAtGoal() && flywheel.isAtGoal() && turret.isAtGoal()) {
          state = ShooterState.SHOOT;
        }
      } else {
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

  public double getHoodPositionDegrees() {
    return hood.getPositionDegrees();
  }
}
