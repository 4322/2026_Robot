package frc.robot.subsystems.shooter.shootingManager;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ShootingParameters;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.subsystems.shooter.areaManager.AreaManager.Zone;

public class ShootingManager {

  public record ShootingSolution(double flywheelSpeed, double hoodAngle) {}

  public enum ShootingTargets {
    HUB,
    ALLIANCE_RIGHT,
    ALLIANCE_LEFT,
    NEUTRAL_LEFT,
    NEUTRAL_RIGHT
  }

  public static ShootingSolution getShootingSolution(Translation2d robotPosition, Translation2d robotVelocity) {
    // Project future position
    Translation2d futurePos = robotPosition.plus(robotVelocity.times(Constants.ShootingManager.latencyCompensation));

    // Get target vector
    Translation2d goalPosition = getShootingTarget(robotPosition);
    Translation2d toGoal = goalPosition.minus(futurePos);
    double distance = toGoal.getNorm();
    Translation2d targetDirection = toGoal.div(distance);

    // Get velocity
    ShootingParameters baseline = Constants.ShootingManager.shooterMap.get(distance);
    double baselineVelocity = distance / baseline.getTimeOfFlightSec();

    // Build target velocity vector
    Translation2d targetVelocity = targetDirection.times(baselineVelocity);

    // Compensate for robot velocity
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

    // Get results
    Rotation2d turretAngle = shotVelocity.getAngle();
    double requiredVelocity = shotVelocity.getNorm();

    // Use table in reverse: velocity → effective distance → RPM
        double effectiveDistance = velocityToEffectiveDistance(requiredVelocity);

    return getDynamicShootingSolution(effectiveDistance, requiredVelocity);
  }

  private static ShootingSolution getDynamicShootingSolution(double distance, double requiredVelocity) {
    ShootingParameters baseline = Constants.ShootingManager.shooterMap.get(distance);
    double baselineVelocity = distance / baseline.getTimeOfFlightSec();
    double velocityRatio = requiredVelocity / baselineVelocity;

    double rpmFactor = Math.sqrt(velocityRatio);
    double hoodFactor = Math.sqrt(velocityRatio);

    double adjustedRPM =
        baseline.getFlywheelRPM()
            * rpmFactor;

    double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.getHoodAngleDeg()));

    double targetHorizFromHood = baselineVelocity * hoodFactor;
    double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0.0, 1.0);
    double adjustedHood = Math.toDegrees(Math.acos(ratio));

    return new ShootingSolution(adjustedRPM, adjustedHood);
  }

  public static double velocityToEffectiveDistance(double velocity) {
    return Constants.ShootingManager.velocityToDistanceMap.get(velocity);
  }

  private static Translation2d getShootingTarget(Translation2d robotPosition) {
    Zone zone = AreaManager.getZoneOfPosition(robotPosition);
    
    if (Robot.alliance == Alliance.Blue) {
      switch (zone) {
        case ALLIANCE_ZONE:
          return Constants.ShootingTargetTranslations.Blue.hubPose;
        case RIGHT_NEUTRAL:
          return Constants.ShootingTargetTranslations.Blue.allianceRightPose;
        case LEFT_NEUTRAL:
          return Constants.ShootingTargetTranslations.Blue.allianceLeftPose;
        case RIGHT_OPPOSITION:
          return Constants.ShootingTargetTranslations.Blue.neutralRightPose;
        case LEFT_OPPOSITION:
          return Constants.ShootingTargetTranslations.Blue.neutralLeftPose;
        default:
          return new Translation2d();
      }
    } else {
      switch (zone) {
        case ALLIANCE_ZONE:
          return Constants.ShootingTargetTranslations.Red.hubPose;
        case RIGHT_NEUTRAL:
          return Constants.ShootingTargetTranslations.Red.allianceRightPose;
        case LEFT_NEUTRAL:
          return Constants.ShootingTargetTranslations.Red.allianceLeftPose;
        case RIGHT_OPPOSITION:
          return Constants.ShootingTargetTranslations.Red.neutralRightPose;
        case LEFT_OPPOSITION:
          return Constants.ShootingTargetTranslations.Red.neutralLeftPose;
        default:
          return new Translation2d();
      }
    }
  }
}
