package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.spindexer.Spindexer;
import frc.robot.subsystems.shooter.tunnel.Tunnel;
import frc.robot.subsystems.shooter.turret.Turret;

public class Shooter extends SubsystemBase {

    
    private enum ShooterState {
        DISABLED,
        UNWIND,
        AUTO_SHOOTING,
        INHIBIT_AUTO_SHOOTING,
        AREA_INHIBIT_AUTO_SHOOTING

    }

    private ShooterState state = ShooterState.DISABLED;
    private ShooterState previousState = ShooterState.DISABLED;

    private Flywheel flywheel;
    private Hood hood;
    private Spindexer spindexer;
    private Tunnel tunnel;
    private Turret turret; 

    private double targetAngle;

    public Shooter(Flywheel flywheel, Hood hood, Spindexer spindexer, Tunnel tunnel, Turret turret) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.spindexer = spindexer;
        this.tunnel = tunnel;
        this.turret = turret;
    }

    @Override
    public void periodic() {
       switch (state) {
            case DISABLED -> {
                if (DriverStation.isEnabled()) {
                    // TODO hood.home();
                }
            }
            case UNWIND -> {
                if (/*turret.isUnwound()*/) {
                    state = previousState;
                    previousState = ShooterState.UNWIND;
                }
            }
            case AUTO_SHOOTING -> {
                if (/*turrent.needsToUnwind()*/) {
                    previousState = state;
                    state = ShooterState.UNWIND;
                }
                if (/*in non-shooting area */) {
                    previousState = state;
                    state = ShooterState.AREA_INHIBIT_AUTO_SHOOTING;
                }
                /*
                Auto shooting logic/code
                
                */
            }
            case INHIBIT_AUTO_SHOOTING -> {
                

            }
            case AREA_INHIBIT_AUTO_SHOOTING -> {

            }
        }
        /* TODO once these are all set up
        flywheel.periodic();
        hood.periodic();
        spindexer.periodic();
        tunnel.periodic();
        turret.periodic();
        */
       Logger.recordOutput("Shooter/State", state.toString());
    }
}
