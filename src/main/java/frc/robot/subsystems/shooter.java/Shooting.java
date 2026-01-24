package frc.robot.subsystems.Shooter.java;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooting extends SubsystemBase {
    private enum ShooterState {
        DISABLED,
        UNWINDING,
        SCORING,
        PASSING,
        INHIBIT

    }

    ShooterState currentState = ShooterState.DISABLED;

    private void periodic() {
       switch (currentState) {
            case DISABLED ->{

            }
            case UNWINDING -> {

            }
            case SCORING ->  {

            }
            case PASSING -> {

            }
            case INHIBIT -> {

            }
        }
    }
}
