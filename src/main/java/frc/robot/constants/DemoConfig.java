package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class DemoConfig {
    // All units in meters
    public static final boolean demoEnabled = true;
    public static final boolean useGeofence = true;

    public static double fieldLength = Units.feetToMeters(40);
    public static double fieldWidth = Units.feetToMeters(20);

    public static double robotMaxLength = 0; // TODO, measure with intake out
    public static double robotMaxWidth = 0;
    public static double robotMaxSide = Math.max(robotMaxLength, robotMaxWidth);

    public class Geofence {
        public static final double margin = 0.8; // TODO tune this
        
        public static final double minX = robotMaxSide / 2;
        public static final double maxX = fieldLength - robotMaxSide / 2;
        public static final double minY = robotMaxSide / 2;
        public static final double maxY = fieldWidth - robotMaxSide / 2;
    }
}
