package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private boolean requestUnwinding;
    private boolean request
;
    private enum ShooterState {
        DISABLED,
        UNWIND,
        AUTO_SHOOTING,
        INHIBIT_AUTO_SHOOTING,
        AREA_INHIBIT_AUTO_SHOOTING

    }

    ShooterState currentState = ShooterState.DISABLED;

    @Override
    public void periodic() {
       switch (currentState) {
            case DISABLED ->{

            }
            case UNWIND ->{

            }
            case AUTO_SHOOTING ->{

            }
            case INHIBIT_AUTO_SHOOTING ->{

            }
            case AREA_INHIBIT_AUTO_SHOOTING ->{

            }
        }
    }
}
