package frc.robot.subsystems.shooter.areaManager;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.DemoConfig;

public class AreaManager {

  public enum Zone {
    LEFT,
    RIGHT,
    UNKNOWN
  }

  public static boolean isShootingArea(Pose2d position) {
    return true;
  }

  public static Zone getZoneOfPosition(Pose2d position) {
    if (DemoConfig.DemoFields.leftZone.contains(position)) {
      return Zone.LEFT;
    } else if (DemoConfig.DemoFields.rightZone.contains(position)) {
      return Zone.RIGHT;
    }
    return Zone.UNKNOWN;
  }
}
