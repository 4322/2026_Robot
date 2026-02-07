package frc.robot.subsystems.shooter.shootingManager;

import frc.robot.constants.Constants.ShootingParameters;

public class ShootingManager {
    public enum ShootingTargets {
        HUB,
        ALLIANCE_RIGHT,
        ALLIANCE_LEFT,
        NEUTRAL_LEFT,
        NEUTRAL_RIGHT
    }

    public static ShootingParameters getShootingParameters(ShootingTargets target) {
        // TODO
        return new ShootingParameters(0, 0, 0);
    }
}
