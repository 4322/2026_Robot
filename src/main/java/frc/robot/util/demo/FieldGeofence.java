package frc.robot.util.demo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.DemoConfig;

public class FieldGeofence {

    public static ChassisSpeeds applyGeofence(ChassisSpeeds fieldSpeeds, Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        double vx = fieldSpeeds.vxMetersPerSecond;
        double vy = fieldSpeeds.vyMetersPerSecond;

        // Basically uses exponential function to reduce speed as robot approaches border
        if (vx < 0) {
            vx *= easeSpeed(x - DemoConfig.Geofence.minX);
        }
        if (vx > 0) {
            vx *= easeSpeed(DemoConfig.Geofence.maxX - x);
        }
        if (vy < 0) {
            vy *= easeSpeed(y - DemoConfig.Geofence.minY);
        }
        if (vy > 0) {
            vy *= easeSpeed(DemoConfig.Geofence.maxY - y);
        }
        return new ChassisSpeeds(vx, vy, fieldSpeeds.omegaRadiansPerSecond); // We leave rotation unchanged
    }
    
    private static double easeSpeed(double distanceToWall) {
        if (distanceToWall >= DemoConfig.Geofence.margin) {
            return 1.0;
        }
        if (distanceToWall <= 0) {
            return 0.0;
        }
        double x = distanceToWall / DemoConfig.Geofence.margin;
        return 1.0 - Math.pow(1.0 - x, 2); // TODO Need to check to see if this works okay
    }

}
