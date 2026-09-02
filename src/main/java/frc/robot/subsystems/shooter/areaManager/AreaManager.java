package frc.robot.subsystems.shooter.areaManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.constants.DemoConfig;
import frc.robot.constants.FieldConstants;

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
