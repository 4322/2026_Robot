package frc.robot.subsystems.shooter.firingManager;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
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
    Translation2d goalPosition = getShootingTarget(turretPosition);
    Translation2d toGoal = goalPosition.minus(turretPosition);
    double distance = toGoal.getNorm();
    Translation2d targetDirection = toGoal.div(distance);
    Logger.recordOutput("FiringManager/targetPosition", new Pose2d(goalPosition, new Rotation2d()));
    Logger.recordOutput("FiringManager/isScoring", isScoring);
    Logger.recordOutput("FiringManager/distance", distance);

    if (Constants.firingManager == Constants.SubsystemMode.TUNING) {
      Logger.recordOutput("FiringManager/requestedTuning/flywheelSpeedRPM", flywheelSpeedRPM.get());
      Logger.recordOutput("FiringManager/requestedTuning/hoodAngle", hoodAngle.get());
      Logger.recordOutput("FiringManager/requestedTuning/tunnelSpeedRPS", tunnelSpeedRPS.get());
      Logger.recordOutput("FiringManager/requestedTuning/indexerSpeedRPS", indexerSpeedRPS.get());
      return new FiringSolution(
          flywheelSpeedRPM.get(),
          hoodAngle.get(),
          adjustForTurretLock(targetDirection.getAngle().getDegrees()),
          tunnelSpeedRPS.get(),
          indexerSpeedRPS.get());
    }

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
    toGoal = goalPosition.minus(futurePos);
    distance = toGoal.getNorm();
    targetDirection = toGoal.div(distance);
    Logger.recordOutput("FiringManager/futurePos", new Pose2d(futurePos, new Rotation2d()));
    Logger.recordOutput("FiringManager/adjustedDistance", distance);

    // Get FiringParameters based on distance
    FiringParameters baseline =
        isScoring
            ? Constants.FiringManager.firingMapScoring.get(distance)
            : Constants.FiringManager.firingMapPassing.get(distance);

    if (!Constants.shootOnTheMoveEnabled) {
      return new FiringSolution(
          baseline.getFlywheelRPM(),
          baseline.getHoodAngleDeg(),
          adjustForTurretLock(targetDirection.getAngle().getDegrees()),
          baseline.getTunnelRPS(),
          baseline.getIndexerRPS());
    }

    // Build target velocity vector
    double baselineVelocity = distance / baseline.getTimeOfFlightSec();
    Translation2d targetVelocity = targetDirection.times(baselineVelocity);

    // Compensate for robot velocity
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);
    Logger.recordOutput(
        "FiringManager/calculatedShootingTarget",
        new Pose2d(turretPosition.plus(shotVelocity), new Rotation2d()));

    // Get turret angle based on angle of target velocity vector
    Rotation2d turretAngle = new Rotation2d();
    try {
      turretAngle = shotVelocity.getAngle();
    } catch (Exception e) {
    }

    double requiredVelocity = shotVelocity.getNorm();
    Logger.recordOutput("FiringManager/requiredVelocity", requiredVelocity);

    // Use table in reverse: get effective distance from required velocity
    double effectiveDistance = velocityToEffectiveDistance(requiredVelocity, isScoring);
    Logger.recordOutput("FiringManager/effectiveDistance", effectiveDistance);

    return getHybridFiringSolution(
        effectiveDistance,
        requiredVelocity,
        turretAngle,
        baseline.getTunnelRPS(),
        baseline.getIndexerRPS(),
        isScoring);
  }

  /**
   * Achieve target horizontal velocity by adjusting both flywheel RPM and hood angle (Equal
   * combination of both)
   */
  private static FiringSolution getHybridFiringSolution(
      double distance,
      double requiredVelocity,
      Rotation2d turretAngle,
      double tunnelSpeedRPS,
      double indexerSpeedRPS,
      boolean isScoring) {
    FiringParameters baseline =
        isScoring
            ? Constants.FiringManager.firingMapScoring.get(distance)
            : Constants.FiringManager.firingMapPassing.get(distance);
    double baselineVelocity = distance / baseline.getTimeOfFlightSec();
    double velocityRatio = requiredVelocity / baselineVelocity;

    // Split correction; sqrt gives equal contribution from both RPM and hood change
    double rpmFactor = Math.sqrt(velocityRatio);
    double hoodFactor = Math.sqrt(velocityRatio);

    Logger.recordOutput("FiringManager/rpmFactor", rpmFactor);
    Logger.recordOutput("FiringManager/hoodFactor", hoodFactor);

    // Apply RPM scaling
    double adjustedRPM = baseline.getFlywheelRPM() * rpmFactor;

    double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.getHoodAngleDeg()));

    // Apply hood adjustment
    double targetHorizFromHood = baselineVelocity * hoodFactor;
    double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0.0, 1.0);
    double adjustedHood = Math.toDegrees(Math.acos(ratio));

    Logger.recordOutput("FiringManager/FiringSolution/adjustedRPM", adjustedRPM);
    Logger.recordOutput("FiringManager/FiringSolution/adjustedHood", adjustedHood);
    Logger.recordOutput("FiringManager/FiringSolution/turretAngle", turretAngle.getDegrees());
    return new FiringSolution(
        adjustedRPM,
        adjustedHood,
        adjustForTurretLock(turretAngle.getDegrees()),
        tunnelSpeedRPS,
        indexerSpeedRPS);
  }

  private static double adjustForTurretLock(double turretDeg) {
    if (Constants.turretLocked) {
      return Rotation2d.fromDegrees(turretDeg).rotateBy(Rotation2d.kCW_Pi_2).getDegrees();
    } else {
      return turretDeg;
    }
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
