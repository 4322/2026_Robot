package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.spindexer.Spindexer;
import frc.robot.subsystems.shooter.tunnel.Tunnel;
import frc.robot.subsystems.shooter.turret.Turret;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  public enum ShooterState {
    DISABLED,
    IDLE,
    UNWIND,
    PRESHOOT,
    SHOOT,
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
      case IDLE -> {
        spindexer.requestIdle();
        tunnel.requestIdle();
        flywheel.requestIdle();
      }
      case UNWIND -> {
        spindexer.requestIdle();
        tunnel.requestIdle();
        flywheel.requestIdle();
        turret.preemptiveUnwind();
      }
      case PRESHOOT -> {
        flywheel.requestShoot(Constants.Flywheel.shootingMechanismRPS);
      }
      case SHOOT -> {
        flywheel.requestShoot(Constants.Flywheel.shootingMechanismRPS);
        tunnel.requestIndex(Constants.Tunnel.dynamicVelocity ? Constants.Tunnel.dynamicVelocityPercent * flywheel.getVelocity() 
            : Constants.Tunnel.indexingMechanismRotationsPerSec);
        spindexer.requestIndex(Constants.Spindexer.dynamicVelocity ? Constants.Spindexer.dynamicVelocityPercent * tunnel.getVelocity() 
            : Constants.Spindexer.indexingMechanismRotationsPerSec);
        
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

  public boolean isRewindComplete() {
    return false; // TODO turret.isUnwound();
  }

  public boolean isMechanismsAtSpeed() {
    return flywheel.atTargetVelocity() && tunnel.isAtSpeed();
  }

  public boolean isFlywheelAtSpeed() {
    return flywheel.atTargetVelocity();
  }

  public boolean isHoodAtAngle() {
    return false; // TODO
  }

  public void setState(ShooterState newState) {
    if (newState != state) {
      previousState = state;
      state = newState;
    }
  }
}
