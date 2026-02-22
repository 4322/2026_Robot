package frc.robot.subsystems.shooter.firingManager;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.FiringParameters;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.subsystems.shooter.areaManager.AreaManager.Zone;
import org.littletonrobotics.junction.Logger;

public class FiringManager {

  public record FiringSolution(double flywheelSpeedRPM, double hoodAngle, double turretAngleDeg, double tunnelSpeedRPS, double indexerSpeedRPS) {}

  public enum FiringTargets {
    HUB,
    ALLIANCE_RIGHT,
    ALLIANCE_LEFT,
    NEUTRAL_LEFT,
    NEUTRAL_RIGHT
  }

  public static FiringSolution getFiringSolution(
      Translation2d robotPosition, Translation2d robotVelocity, boolean isScoring) {

    // Project future position
    double latencyCompensation = isScoring ? Constants.FiringManager.latencyCompensationScoring : Constants.FiringManager.latencyCompensationPassing;
    Translation2d futurePos =
        robotPosition.plus(robotVelocity.times(latencyCompensation));
    
    Logger.recordOutput("FiringManager/futurePos", futurePos);
    // Get target vector
    Translation2d goalPosition = getShootingTarget(robotPosition);
    Logger.recordOutput("FiringManager/goalPosition", goalPosition);
    Translation2d toGoal = goalPosition.minus(futurePos);
    double distance = toGoal.getNorm();
    Logger.recordOutput("FiringManager/distance", distance);
    Translation2d targetDirection = toGoal.div(distance);

    // Get velocity
    FiringParameters baseline = isScoring ? Constants.FiringManager.firingMapScoring.get(distance) : Constants.FiringManager.firingMapPassing.get(distance);
    double baselineVelocity = distance / baseline.getTimeOfFlightSec();

    // Build target velocity vector
    Translation2d targetVelocity = targetDirection.times(baselineVelocity);

    // Compensate for robot velocity
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

    // Get results
    Rotation2d turretAngle = shotVelocity.getAngle();
    double requiredVelocity = shotVelocity.getNorm();
    Logger.recordOutput("FiringManager/requiredVelocity", requiredVelocity);

    // Use table in reverse: velocity -> effective distance → RPM
    double effectiveDistance = velocityToEffectiveDistance(requiredVelocity, isScoring);
    Logger.recordOutput("FiringManager/effectiveDistance", effectiveDistance);

    // TODO implement tunnel/indexer speed
    return getHybridFiringSolution(effectiveDistance, requiredVelocity, turretAngle, Constants.Tunnel.indexingMechanismRotationsPerSec, Constants.Spindexer.indexingMechanismRotationsPerSec, isScoring);
  }

  private static FiringSolution getHybridFiringSolution(
      double distance, double requiredVelocity, Rotation2d turretAngle, double tunnelSpeedRPS, double indexerSpeedRPS, boolean isScoring) {
    FiringParameters baseline = isScoring ? Constants.FiringManager.firingMapScoring.get(distance) : Constants.FiringManager.firingMapPassing.get(distance);
    double baselineVelocity = distance / baseline.getTimeOfFlightSec();
    double velocityRatio = requiredVelocity / baselineVelocity;

    double rpmFactor = Math.sqrt(velocityRatio);
    Logger.recordOutput("FiringManager/rpmFactor", rpmFactor);
    double hoodFactor = Math.sqrt(velocityRatio);
    Logger.recordOutput("FiringManager/hoodFactor", hoodFactor);

    double adjustedRPM = baseline.getFlywheelRPM() * rpmFactor;

    double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.getHoodAngleDeg()));

    double targetHorizFromHood = baselineVelocity * hoodFactor;
    double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0.0, 1.0);
    double adjustedHood = Math.toDegrees(Math.acos(ratio));

    Logger.recordOutput("FiringManager/FiringSolution/adjustedRPM", adjustedRPM);
    Logger.recordOutput("FiringManager/FiringSolution/adjustedHood", adjustedHood);
    Logger.recordOutput("FiringManager/FiringSolution/turretAngle", turretAngle.getDegrees());
    return new FiringSolution(adjustedRPM, adjustedHood, turretAngle.getDegrees(), tunnelSpeedRPS, indexerSpeedRPS);
  }

  public static double velocityToEffectiveDistance(double velocity, boolean isScoring) {
    return isScoring ? Constants.FiringManager.velocityToDistanceMapScoring.get(velocity) : Constants.FiringManager.velocityToDistanceMapPassing.get(velocity);
  }

  private static Translation2d getShootingTarget(Translation2d robotPosition) {
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
