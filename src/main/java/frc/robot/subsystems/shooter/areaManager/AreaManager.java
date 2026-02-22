package frc.robot.subsystems.shooter.areaManager;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;

public class AreaManager {

  public enum Zone {
    ALLIANCE_ZONE,
    LEFT_OPPOSITION,
    RIGHT_OPPOSITION,
    LEFT_NEUTRAL,
    RIGHT_NEUTRAL,
    LEFT_TRENCH,
    RIGHT_TRENCH,
    LEFT_OPPOSITION_TRENCH,
    RIGHT_OPPOSITION_TRENCH,
    LEFT_STOP_SHOOT,
    RIGHT_STOP_SHOOT,
    LEFT_OPPOSITION_STOP_SHOOT,
    RIGHT_OPPOSITION_STOP_SHOOT
  }

  private static Zone zone = Zone.ALLIANCE_ZONE;

  public static boolean isShootingArea(Translation2d position) {
    if (Robot.alliance == Alliance.Blue) {
      return !(FieldConstants.Neutral.backBlueHub.contains(position)
          || FieldConstants.Red.frontOfHub.contains(position)
          || FieldConstants.Blue.stopShootLeft.contains(position)
          || FieldConstants.Blue.stopShootRight.contains(position)
          || FieldConstants.Red.stopShootLeft.contains(position)
          || FieldConstants.Red.stopShootRight.contains(position));
    } else {
      return !(FieldConstants.Neutral.backRedHub.contains(position)
          || FieldConstants.Blue.frontOfHub.contains(position)
          || FieldConstants.Blue.stopShootLeft.contains(position)
          || FieldConstants.Blue.stopShootRight.contains(position)
          || FieldConstants.Red.stopShootLeft.contains(position)
          || FieldConstants.Red.stopShootRight.contains(position));
    }
  }

  public static boolean isTrench(Translation2d position) {
    return ((FieldConstants.Blue.trenchLeft.contains(position)
        || FieldConstants.Blue.trenchRight.contains(position)
        || FieldConstants.Red.trenchLeft.contains(position)
        || FieldConstants.Red.trenchRight.contains(position)));
  }

  public static boolean isHoodDangerZone(Translation2d position) {
    return FieldConstants.Blue.stopShootLeft.contains(position)
        || FieldConstants.Blue.stopShootRight.contains(position)
        || FieldConstants.Red.stopShootLeft.contains(position)
        || FieldConstants.Red.stopShootRight.contains(position);
  }

  public static Zone getZoneOfPosition(Translation2d position) {
    if (Robot.alliance == Alliance.Blue) {
      if (FieldConstants.Blue.allianceZone.contains(position)) {
        return Zone.ALLIANCE_ZONE;
      } else if (FieldConstants.Red.rightAllianceZone.contains(position)) {
        return Zone.RIGHT_OPPOSITION;
      } else if (FieldConstants.Red.leftAllianceZone.contains(position)) {
        return Zone.LEFT_OPPOSITION;
      } else if (FieldConstants.Neutral.rightNeutral.contains(position)) {
        return Zone.RIGHT_NEUTRAL;
      } else if (FieldConstants.Neutral.leftNeutral.contains(position)) {
        return Zone.LEFT_NEUTRAL;
      } else if (FieldConstants.Blue.trenchLeft.contains(position)) {
        return Zone.LEFT_TRENCH;
      } else if (FieldConstants.Blue.trenchRight.contains(position)) {
        return Zone.RIGHT_TRENCH;
      } else if (FieldConstants.Red.trenchLeft.contains(position)) {
        return Zone.LEFT_OPPOSITION_TRENCH;
      } else if (FieldConstants.Red.trenchRight.contains(position)) {
        return Zone.RIGHT_OPPOSITION_TRENCH;
      } else if (FieldConstants.Blue.stopShootLeft.contains(position)) {
        return Zone.LEFT_STOP_SHOOT;
      } else if (FieldConstants.Blue.stopShootRight.contains(position)) {
        return Zone.RIGHT_STOP_SHOOT;
      } else if (FieldConstants.Red.stopShootLeft.contains(position)) {
        return Zone.LEFT_OPPOSITION_STOP_SHOOT;
      } else if (FieldConstants.Red.stopShootRight.contains(position)) {
        return Zone.RIGHT_OPPOSITION_STOP_SHOOT;
      }
    } else {
      if (FieldConstants.Red.allianceZone.contains(position)) {
        return Zone.ALLIANCE_ZONE;
      } else if (FieldConstants.Blue.rightAllianceZone.contains(position)) {
        return Zone.RIGHT_OPPOSITION;
      } else if (FieldConstants.Blue.leftAllianceZone.contains(position)) {
        return Zone.LEFT_OPPOSITION;
      } else if (FieldConstants.Neutral.rightNeutral.contains(position)) {
        return Zone.RIGHT_NEUTRAL;
      } else if (FieldConstants.Neutral.leftNeutral.contains(position)) {
        return Zone.LEFT_NEUTRAL;
      } else if (FieldConstants.Red.trenchLeft.contains(position)) {
        return Zone.LEFT_TRENCH;
      } else if (FieldConstants.Red.trenchRight.contains(position)) {
        return Zone.RIGHT_TRENCH;
      } else if (FieldConstants.Blue.trenchLeft.contains(position)) {
        return Zone.LEFT_OPPOSITION_TRENCH;
      } else if (FieldConstants.Blue.trenchRight.contains(position)) {
        return Zone.RIGHT_OPPOSITION_TRENCH;
      } else if (FieldConstants.Red.stopShootLeft.contains(position)) {
        return Zone.LEFT_STOP_SHOOT;
      } else if (FieldConstants.Red.stopShootRight.contains(position)) {
        return Zone.RIGHT_STOP_SHOOT;
      } else if (FieldConstants.Blue.stopShootLeft.contains(position)) {
        return Zone.LEFT_OPPOSITION_STOP_SHOOT;
      } else if (FieldConstants.Blue.stopShootRight.contains(position)) {
        return Zone.RIGHT_OPPOSITION_STOP_SHOOT;
      }
    }
    return null;
  }

  // Returns true if positions are same larger zone, ignoring right/left designations (ex. right
  // neutral and left neutral -> true)
  public static boolean isSameCompleteZone(Translation2d position1, Translation2d position2) {
    Zone zone1 = getZoneOfPosition(position1);
    Zone zone2 = getZoneOfPosition(position2);

    if (zone1 == Zone.ALLIANCE_ZONE && zone2 == Zone.ALLIANCE_ZONE) {
      return true;
    } else if ((zone1 == Zone.LEFT_OPPOSITION || zone1 == Zone.RIGHT_OPPOSITION)
        && (zone2 == Zone.LEFT_OPPOSITION || zone2 == Zone.RIGHT_OPPOSITION)) {
      return true;
    } else if ((zone1 == Zone.LEFT_NEUTRAL || zone1 == Zone.RIGHT_NEUTRAL)
        && (zone2 == Zone.LEFT_NEUTRAL || zone2 == Zone.RIGHT_NEUTRAL)) {
      return true;
    } else if ((zone1 == Zone.LEFT_TRENCH || zone1 == Zone.RIGHT_TRENCH)
        && (zone2 == Zone.LEFT_TRENCH || zone2 == Zone.RIGHT_TRENCH)) {
      return true;
    } else if ((zone1 == Zone.LEFT_OPPOSITION_TRENCH || zone1 == Zone.RIGHT_OPPOSITION_TRENCH)
        && (zone2 == Zone.LEFT_OPPOSITION_TRENCH || zone2 == Zone.RIGHT_OPPOSITION_TRENCH)) {
      return true;
    } else if ((zone1 == Zone.LEFT_STOP_SHOOT || zone1 == Zone.RIGHT_STOP_SHOOT)
        && (zone2 == Zone.LEFT_STOP_SHOOT || zone2 == Zone.RIGHT_STOP_SHOOT)) {
      return true;
    } else if ((zone1 == Zone.LEFT_OPPOSITION_STOP_SHOOT
            || zone1 == Zone.RIGHT_OPPOSITION_STOP_SHOOT)
        && (zone2 == Zone.LEFT_OPPOSITION_STOP_SHOOT
            || zone2 == Zone.RIGHT_OPPOSITION_STOP_SHOOT)) {
      return true;
    }
    return false;
  }
}
