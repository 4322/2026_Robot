package frc.robot.util.demo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.DemoConfig;
import org.littletonrobotics.junction.Logger;

public class FieldGeofence {

  public static ChassisSpeeds applyGeofence(ChassisSpeeds fieldSpeeds, Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    double vy = fieldSpeeds.vxMetersPerSecond;
    double vx = fieldSpeeds.vyMetersPerSecond;
    Logger.recordOutput("FieldGeofence/prevVX", vx);
    Logger.recordOutput("FieldGeofence/prevVY", vy);
    Logger.recordOutput("FieldGeofence/prevFieldSpeeds", fieldSpeeds);

    Logger.recordOutput("DistanceToTopWall", DemoConfig.DemoFields.maxY - y);
    Logger.recordOutput("DistanceToBottomWall", y - DemoConfig.DemoFields.minY);
    Logger.recordOutput("DistanceToLeftWall", x - DemoConfig.DemoFields.minX);
    Logger.recordOutput("DistanceToRightWall", DemoConfig.DemoFields.maxX - x);
    // Basically uses exponential function to reduce speed as robot approaches border
    if (vx > 0) {
      vx *= easeSpeed(x - DemoConfig.DemoFields.minX);
      Logger.recordOutput("FieldGeofence/easeSpeedVX", easeSpeed(x - DemoConfig.DemoFields.minX));
      Logger.recordOutput("FieldGeofence/postVX", vx);
    }
    if (vx < 0) {
      vx *= easeSpeed(DemoConfig.DemoFields.maxX - x);
      Logger.recordOutput("FieldGeofence/easeSpeedVX", easeSpeed(DemoConfig.DemoFields.maxX - x));
      Logger.recordOutput("FieldGeofence/postVX", vx);
    }
    if (vy < 0) {
      vy *= easeSpeed(y - DemoConfig.DemoFields.minY);
      Logger.recordOutput("FieldGeofence/easeSpeedVY", easeSpeed(y - DemoConfig.DemoFields.minY));
      Logger.recordOutput("FieldGeofence/postVY", vy);
    }
    if (vy > 0) {
      vy *= easeSpeed(DemoConfig.DemoFields.maxY - y);
      Logger.recordOutput("FieldGeofence/easeSpeedVY", easeSpeed(DemoConfig.DemoFields.maxY - y));
      Logger.recordOutput("FieldGeofence/postVY", vy);
    }
    Logger.recordOutput(
        "FieldGeofence/postFieldSpeeds",
        new ChassisSpeeds(vy, vx, fieldSpeeds.omegaRadiansPerSecond));
    return new ChassisSpeeds(
        vy, vx, fieldSpeeds.omegaRadiansPerSecond); // We leave rotation unchanged
  }

  private static double easeSpeed(double distanceToWall) {
    if (distanceToWall >= DemoConfig.DemoFields.margin) {
      return 1.0;
    }
    if (distanceToWall <= 0) {
      return 0.0;
    }
    double x = distanceToWall / DemoConfig.DemoFields.margin;
    return 1.0 - Math.pow(1.0 - x, 2); // TODO Need to check to see if this works okay
  }
}
