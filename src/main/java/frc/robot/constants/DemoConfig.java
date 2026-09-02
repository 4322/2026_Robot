package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;

public class DemoConfig {
    // All units in meters
    public static final boolean demoEnabled = true;
    public static final boolean useGeofence = true;

    public static final boolean maxDriveSpeedOverride = true; // Set to true to override the normal speed (4.775 m/s)
    public static final LinearVelocity maxDriveSpeed = MetersPerSecond.of(1);


    // Config stuff that only needs to be set once per demo

    public static double fieldLength = Units.feetToMeters(40);
    public static double fieldWidth = Units.feetToMeters(20);

    // Stuff that shouldn't change in between demos
    
    public static double robotMaxLength = 0; // TODO, measure with intake out
    public static double robotMaxWidth = 0;
    public static double robotMaxSide = Math.max(robotMaxLength, robotMaxWidth);


    // Other classes/utils

    public class Geofence {
        public static final double margin = 0.8; // TODO tune this

        public static final double minX = robotMaxSide / 2;
        public static final double maxX = fieldLength - robotMaxSide / 2;
        public static final double minY = robotMaxSide / 2;
        public static final double maxY = fieldWidth - robotMaxSide / 2;
    }
}
