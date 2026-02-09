package frc.robot.subsystems.shooter.shootingManager;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ShootingParameters;

public class ShootingManager {

  public static class ShootingSolution {
    private double flywheelSpeed;
    private double hoodAngle;

    public ShootingSolution(double flywheelSpeed, double hoodAngle) {
      this.flywheelSpeed = flywheelSpeed;
      this.hoodAngle = hoodAngle;
    }

    public double getFlywheelSpeed() {
      return flywheelSpeed;
    }

    public double getHoodAngle() {
      return hoodAngle;
    }
  }

  public enum ShootingTargets {
    HUB,
    ALLIANCE_RIGHT,
    ALLIANCE_LEFT,
    NEUTRAL_LEFT,
    NEUTRAL_RIGHT
  }

  public static ShootingSolution getShootingSolution(ShootingTargets target, Pose2d robotPose) {
    // TODO

    double targetX = getShootingTarget(target).getX() - robotPose.getX();
    double targetY = getShootingTarget(target).getY() - robotPose.getY();
    Translation2d targetPosition = new Translation2d(targetX, targetY);

    double distance = targetPosition.getNorm();

    return new ShootingSolution(0, 0);
  }

  private static ShootingSolution getMapShootingSolution(double distance, double requiredVelocity) {
    ShootingParameters baseline = Constants.ShootingManager.shooterMap.get(distance);
    double baselineVelocity = distance / baseline.getTimeOfFlightSec();
    double velocityRatio = requiredVelocity / baselineVelocity;

    double rpmFactor = Math.sqrt(velocityRatio);
    double hoodFactor = Math.sqrt(velocityRatio);

    double adjustedRPM =
        baseline.getFlywheelRPS()
            * rpmFactor; // TODO check if difference in rpm / rps will cause issues

    double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.getHoodAngleDeg()));

    double targetHorizFromHood = baselineVelocity * hoodFactor;
    double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0.0, 1.0);
    double adjustedHood = Math.toDegrees(Math.acos(ratio));

    return new ShootingSolution(adjustedRPM, adjustedHood);
  }

  private static Pose2d getShootingTarget(ShootingTargets target) {
    if (Robot.alliance == Alliance.Blue) {
      switch (target) {
        case HUB:
          return Constants.ShootingTargetPoses.Blue.hubPose;
        case ALLIANCE_RIGHT:
          return Constants.ShootingTargetPoses.Blue.allianceRightPose;
        case ALLIANCE_LEFT:
          return Constants.ShootingTargetPoses.Blue.allianceLeftPose;
        case NEUTRAL_RIGHT:
          return Constants.ShootingTargetPoses.Blue.neutralRightPose;
        case NEUTRAL_LEFT:
          return Constants.ShootingTargetPoses.Blue.neutralLeftPose;
        default:
          return new Pose2d();
      }
    } else {
      switch (target) {
        case HUB:
          return Constants.ShootingTargetPoses.Red.hubPose;
        case ALLIANCE_RIGHT:
          return Constants.ShootingTargetPoses.Red.allianceRightPose;
        case ALLIANCE_LEFT:
          return Constants.ShootingTargetPoses.Red.allianceLeftPose;
        case NEUTRAL_RIGHT:
          return Constants.ShootingTargetPoses.Red.neutralRightPose;
        case NEUTRAL_LEFT:
          return Constants.ShootingTargetPoses.Red.neutralLeftPose;
        default:
          return new Pose2d();
      }
    }
  }
}
