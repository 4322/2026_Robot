package frc.robot.subsystems.shooter.firingManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.DemoConfig;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.subsystems.shooter.areaManager.AreaManager.Zone;
import org.littletonrobotics.junction.Logger;

public class FiringManager {

  public enum FiringTargets {
    HUB,
    ALLIANCE_RIGHT,
    ALLIANCE_LEFT,
    NEUTRAL_LEFT,
    NEUTRAL_RIGHT
  }

  public static Translation2d getShootingTarget(Pose2d robotPosition) {
    Zone zone = AreaManager.getZoneOfPosition(robotPosition);
    switch(zone) {
      case LEFT -> {
        Logger.recordOutput("FiringManager/targetZone", "Left");
        return DemoConfig.DemoFields.leftTarget;
      }
      case RIGHT -> {
        Logger.recordOutput("FiringManager/targetZone", "Right");
        return DemoConfig.DemoFields.rightTarget;
      }
      case UNKNOWN -> {
        Logger.recordOutput("FiringManager/targetZone", "Unknown, targeting center of field");
        return DemoConfig.DemoFields.centerTarget;
      }
      default -> {
        Logger.recordOutput("FiringManager/targetZone", "Default, targeting center of field");
        return DemoConfig.DemoFields.centerTarget;
      }
    }
  }
}
