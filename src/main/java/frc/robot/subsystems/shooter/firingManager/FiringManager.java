package frc.robot.subsystems.shooter.firingManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.FiringParameters;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.subsystems.shooter.areaManager.AreaManager.Zone;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class FiringManager {

  public record FiringSolution(
      double flywheelSpeedRPS,
      double hoodAngle,
      double turretAngleDeg,
      double tunnelSpeedRPS,
      double indexerSpeedRPS) {}

  public enum FiringTargets {
    HUB,
    ALLIANCE_RIGHT,
    ALLIANCE_LEFT,
    NEUTRAL_LEFT,
    NEUTRAL_RIGHT
  }

  private static final LoggedTunableNumber flywheelSpeedRPM =
      new LoggedTunableNumber("FiringManager/flywheelSpeedRPM", 0);
  private static final LoggedTunableNumber hoodAngle =
      new LoggedTunableNumber("FiringManager/hoodAngle", 0);
  private static final LoggedTunableNumber tunnelSpeedRPS =
      new LoggedTunableNumber("FiringManager/tunnelSpeedRPS", 0);
  private static final LoggedTunableNumber indexerSpeedRPS =
      new LoggedTunableNumber("FiringManager/indexerSpeedRPS", 0);

  public static FiringSolution getFiringSolution(
      Translation2d turretPosition, Translation2d robotVelocity, boolean isScoring) {

    Translation2d givenTargetPosition = getShootingTarget(turretPosition);

    // Project future position based on velocity and latency compensation
    double latencyCompensation = 0;
    if (Constants.shootOnTheMoveEnabled) {
      if (isScoring) {
        latencyCompensation = Constants.FiringManager.latencyCompensationScoring;
      } else {
        latencyCompensation = Constants.FiringManager.latencyCompensationPassing;
      }
    }

    Translation2d futurePos = turretPosition.plus(robotVelocity.times(latencyCompensation));
    Logger.recordOutput("FiringManager/futurePos", new Pose2d(futurePos, new Rotation2d()));

    // Get target vector
    Translation2d vectorToGoal = givenTargetPosition.minus(futurePos);

    // aim slightly behind the center of the hub so we don't hit the front
    if (AreaManager.getZoneOfPosition(turretPosition) == Zone.ALLIANCE_ZONE) {
      givenTargetPosition =
          givenTargetPosition.plus(
              new Translation2d(Units.inchesToMeters(3), 0).rotateBy(vectorToGoal.getAngle()));
      vectorToGoal = givenTargetPosition.minus(turretPosition);
    }

    Logger.recordOutput("FiringManager/vectorToGoal", new Pose2d(vectorToGoal, new Rotation2d()));

    double distanceToGivenTarget = vectorToGoal.getNorm();
    Translation2d targetDirection = vectorToGoal.div(distanceToGivenTarget);

    Logger.recordOutput("FiringManager/distanceToGivenTarget", distanceToGivenTarget);
    Logger.recordOutput(
        "FiringManager/targetDirection", new Pose2d(turretPosition, targetDirection.getAngle()));

    // Get FiringParameters based on distance
    FiringParameters baseline =
        isScoring
            ? Constants.FiringManager.firingMapScoring.get(distanceToGivenTarget)
            : Constants.FiringManager.firingMapPassing.get(distanceToGivenTarget);

    // Build target velocity vector
    double baselineVelocity = distanceToGivenTarget / baseline.getTimeOfFlightSec();
    Translation2d targetVelocity = targetDirection.times(baselineVelocity);

    // Compensate for robot velocity
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

    // Get turret angle based on angle of target velocity vector
    Rotation2d turretAngle = new Rotation2d();
    try {
      turretAngle = shotVelocity.getAngle();
    } catch (Exception e) {
      DriverStation.reportError(
          "Error calculating turret angle in firingManager: " + e.getMessage(), false);
    }

    Logger.recordOutput(
        "FiringManager/calculatedShootingTarget",
        new Pose2d(turretPosition.plus(shotVelocity), new Rotation2d()));

    double requiredVelocity = shotVelocity.getNorm();
    Logger.recordOutput("FiringManager/requiredVelocity", requiredVelocity);

    // Use table in reverse: get effective distance from required velocity
    double effectiveDistance = velocityToEffectiveDistance(requiredVelocity, isScoring);
    FiringParameters solutionParameters =
        isScoring
            ? Constants.FiringManager.firingMapScoring.get(effectiveDistance)
            : Constants.FiringManager.firingMapPassing.get(effectiveDistance);

    FiringSolution solution =
        new FiringSolution(
            solutionParameters.getFlywheelRPM(),
            solutionParameters.getHoodAngleDeg(),
            turretAngle.getDegrees(),
            solutionParameters.getTunnelRPS(),
            solutionParameters.getIndexerRPS());

    Logger.recordOutput("FiringManager/solution/flywheelRPM", solutionParameters.getFlywheelRPM());
    Logger.recordOutput("FiringManager/solution/hoodAngle", solutionParameters.getHoodAngleDeg());
    Logger.recordOutput("FiringManager/solution/tunnelRPS", solutionParameters.getTunnelRPS());
    Logger.recordOutput("FiringManager/solution/indexerRPS", solutionParameters.getIndexerRPS());
    Logger.recordOutput("FiringManager/solution/turretAngle", turretAngle.getDegrees());

    if (Constants.firingManagerMode == Constants.SubsystemMode.TUNING) {
      Logger.recordOutput("FiringManager/requestedTuning/flywheelSpeedRPM", flywheelSpeedRPM.get());
      Logger.recordOutput("FiringManager/requestedTuning/hoodAngle", hoodAngle.get());
      Logger.recordOutput("FiringManager/requestedTuning/tunnelSpeedRPS", tunnelSpeedRPS.get());
      Logger.recordOutput("FiringManager/requestedTuning/indexerSpeedRPS", indexerSpeedRPS.get());
      return new FiringSolution(
          flywheelSpeedRPM.get(),
          hoodAngle.get(),
          adjustForTurretLock(vectorToGoal.getAngle().getDegrees()),
          tunnelSpeedRPS.get(),
          indexerSpeedRPS.get());

    } else if (!Constants.shootOnTheMoveEnabled) {
      if (Constants.turretLocked) {
        return new FiringSolution(
            baseline.getFlywheelRPM(),
            baseline.getHoodAngleDeg(),
            adjustForTurretLock(targetDirection.getAngle().getDegrees()),
            baseline.getTunnelRPS(),
            baseline.getIndexerRPS());
      } else {
        return new FiringSolution(
            baseline.getFlywheelRPM(),
            baseline.getHoodAngleDeg(),
            adjustForTurretLock(givenTargetPosition.getAngle().getDegrees()),
            baseline.getTunnelRPS(),
            baseline.getIndexerRPS());
      }
    }

    return solution;
  }

  private static double adjustForTurretLock(double turretDeg) {
    return Rotation2d.fromDegrees(turretDeg).rotateBy(Rotation2d.kCW_Pi_2).getDegrees();
  }

  public static double velocityToEffectiveDistance(double velocity, boolean isScoring) {
    return isScoring
        ? Constants.FiringManager.velocityToDistanceMapScoring.get(velocity)
        : Constants.FiringManager.velocityToDistanceMapPassing.get(velocity);
  }

  public static Translation2d getShootingTarget(Translation2d robotPosition) {
    Zone zone = AreaManager.getZoneOfPosition(robotPosition);

    if (Robot.alliance == Alliance.Blue) {

      switch (zone) {
        case ALLIANCE_ZONE:
          Logger.recordOutput("FiringManager/targetZone", "Hub");
          return Constants.FiringTargetTranslations.Blue.hubTranslation;
        case RIGHT_NEUTRAL:
          Logger.recordOutput("FiringManager/targetZone", "Alliance Right");
          return Constants.FiringTargetTranslations.Blue.allianceRightTranslation;
        case LEFT_NEUTRAL:
          Logger.recordOutput("FiringManager/targetZone", "Alliance Left");
          return Constants.FiringTargetTranslations.Blue.allianceLeftTranslation;
        case RIGHT_OPPOSITION:
          if (Constants.FiringManager.alwaysTargetAllianceZone) {
            Logger.recordOutput("FiringManager/targetZone", "Alliance Right");
            return Constants.FiringTargetTranslations.Blue.allianceRightTranslation;
          } else {
            Logger.recordOutput("FiringManager/targetZone", "Neutral Right");
            return Constants.FiringTargetTranslations.Blue.neutralRightTranslation;
          }
        case LEFT_OPPOSITION:
          if (Constants.FiringManager.alwaysTargetAllianceZone) {
            Logger.recordOutput("FiringManager/targetZone", "Alliance Left");
            return Constants.FiringTargetTranslations.Blue.allianceLeftTranslation;
          } else {
            Logger.recordOutput("FiringManager/targetZone", "Neutral Left");
            return Constants.FiringTargetTranslations.Blue.neutralLeftTranslation;
          }
        default:
          Logger.recordOutput("FiringManager/targetZone", "Invalid");
          return new Translation2d();
      }
    } else {
      switch (zone) {
        case ALLIANCE_ZONE:
          Logger.recordOutput("FiringManager/targetZone", "Hub");
          return Constants.FiringTargetTranslations.Red.hubTranslation;
        case RIGHT_NEUTRAL:
          Logger.recordOutput("FiringManager/targetZone", "Alliance Right");
          return Constants.FiringTargetTranslations.Red.allianceRightTranslation;
        case LEFT_NEUTRAL:
          Logger.recordOutput("FiringManager/targetZone", "Alliance Left");
          return Constants.FiringTargetTranslations.Red.allianceLeftTranslation;
        case RIGHT_OPPOSITION:
          if (Constants.FiringManager.alwaysTargetAllianceZone) {
            Logger.recordOutput("FiringManager/targetZone", "Alliance Right");
            return Constants.FiringTargetTranslations.Red.allianceRightTranslation;
          } else {
            Logger.recordOutput("FiringManager/targetZone", "Neutral Right");
            return Constants.FiringTargetTranslations.Red.neutralRightTranslation;
          }
        case LEFT_OPPOSITION:
          if (Constants.FiringManager.alwaysTargetAllianceZone) {
            Logger.recordOutput("FiringManager/targetZone", "Alliance Left");
            return Constants.FiringTargetTranslations.Red.allianceLeftTranslation;
          } else {
            Logger.recordOutput("FiringManager/targetZone", "Neutral Left");
            return Constants.FiringTargetTranslations.Red.neutralLeftTranslation;
          }
        default:
          Logger.recordOutput("FiringManager/targetZone", "Invalid");
          return new Translation2d();
      }
    }
  }
}
