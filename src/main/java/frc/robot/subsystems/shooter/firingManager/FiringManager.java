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

public class FiringManager {

  public record FiringSolution(double flywheelSpeedRPM, double hoodAngle, Rotation2d turretAngle) {}

  public enum FiringTargets {
    HUB,
    ALLIANCE_RIGHT,
    ALLIANCE_LEFT,
    NEUTRAL_LEFT,
    NEUTRAL_RIGHT
  }

  public static FiringSolution getFiringSolution(
      Translation2d robotPosition, Translation2d robotVelocity) {
    // Project future position
    Translation2d futurePos =
        robotPosition.plus(robotVelocity.times(Constants.FiringManager.latencyCompensation));

    // Get target vector
    Translation2d goalPosition = getShootingTarget(robotPosition);
    Translation2d toGoal = goalPosition.minus(futurePos);
    double distance = toGoal.getNorm();
    Translation2d targetDirection = toGoal.div(distance);

    // Get velocity
    FiringParameters baseline = Constants.FiringManager.firingMap.get(distance);
    double baselineVelocity = distance / baseline.getTimeOfFlightSec();

    // Build target velocity vector
    Translation2d targetVelocity = targetDirection.times(baselineVelocity);

    // Compensate for robot velocity
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

    // Get results
    Rotation2d turretAngle = shotVelocity.getAngle();
    double requiredVelocity = shotVelocity.getNorm();

    // Use table in reverse: velocity -> effective distance → RPM
    double effectiveDistance = velocityToEffectiveDistance(requiredVelocity);

    return getHybridFiringSolution(effectiveDistance, requiredVelocity, turretAngle);
  }

  private static FiringSolution getHybridFiringSolution(
      double distance, double requiredVelocity, Rotation2d turretAngle) {
    FiringParameters baseline = Constants.FiringManager.firingMap.get(distance);
    double baselineVelocity = distance / baseline.getTimeOfFlightSec();
    double velocityRatio = requiredVelocity / baselineVelocity;

    double rpmFactor = Math.sqrt(velocityRatio);
    double hoodFactor = Math.sqrt(velocityRatio);

    double adjustedRPM = baseline.getFlywheelRPM() * rpmFactor;

    double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.getHoodAngleDeg()));

    double targetHorizFromHood = baselineVelocity * hoodFactor;
    double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0.0, 1.0);
    double adjustedHood = Math.toDegrees(Math.acos(ratio));

    return new FiringSolution(adjustedRPM, adjustedHood, turretAngle);
  }

  public static double velocityToEffectiveDistance(double velocity) {
    return Constants.FiringManager.velocityToDistanceMap.get(velocity);
  }

  private static Translation2d getShootingTarget(Translation2d robotPosition) {
    Zone zone = AreaManager.getZoneOfPosition(robotPosition);

    if (Robot.alliance == Alliance.Blue) {
      switch (zone) {
        case ALLIANCE_ZONE:
          return Constants.FiringTargetTranslations.Blue.hubTranslation;
        case RIGHT_NEUTRAL:
          return Constants.FiringTargetTranslations.Blue.allianceRightTranslation;
        case LEFT_NEUTRAL:
          return Constants.FiringTargetTranslations.Blue.allianceLeftTranslation;
        case RIGHT_OPPOSITION:
          return Constants.FiringTargetTranslations.Blue.neutralRightTranslation;
        case LEFT_OPPOSITION:
          return Constants.FiringTargetTranslations.Blue.neutralLeftTranslation;
        default:
          return new Translation2d();
      }
    } else {
      switch (zone) {
        case ALLIANCE_ZONE:
          return Constants.FiringTargetTranslations.Red.hubTranslation;
        case RIGHT_NEUTRAL:
          return Constants.FiringTargetTranslations.Red.allianceRightTranslation;
        case LEFT_NEUTRAL:
          return Constants.FiringTargetTranslations.Red.allianceLeftTranslation;
        case RIGHT_OPPOSITION:
          return Constants.FiringTargetTranslations.Red.neutralRightTranslation;
        case LEFT_OPPOSITION:
          return Constants.FiringTargetTranslations.Red.neutralLeftTranslation;
        default:
          return new Translation2d();
      }
    }
  }
}
