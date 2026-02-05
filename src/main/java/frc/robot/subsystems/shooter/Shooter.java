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
    PRESHOOT, // Flywheel gets up to speed; Turret aims
    SHOOT, // Spindexer and tunnel get up to speed
  }

  private ShooterState state = ShooterState.DISABLED;

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
          state = ShooterState.IDLE;
        }
      }
      case IDLE -> {
        spindexer.requestIdle();
        if (spindexer.isStopped()) {
          tunnel.requestIdle();
          flywheel.requestIdle();
        }
      }
      case UNWIND -> {
        spindexer.requestIdle();
        if (spindexer.isStopped()) {
          tunnel.requestIdle();
          if (tunnel.isStopped()) {
            turret.preemptiveUnwind();
          }
        }
      }
      case PRESHOOT -> {
        flywheel.requestShoot(Constants.Flywheel.shootingMechanismRPS);
        // TODO Turret request position here
      }
      case SHOOT -> {
        flywheel.requestShoot(Constants.Flywheel.shootingMechanismRPS);
        tunnel.requestIndex(
            Constants.Tunnel.dynamicVelocity
                ? Constants.Tunnel.dynamicVelocityPercent * flywheel.getVelocity()
                : Constants.Tunnel.indexingMechanismRotationsPerSec);
        spindexer.requestIndex(
            Constants.Spindexer.dynamicVelocity
                ? Constants.Spindexer.dynamicVelocityPercent * tunnel.getVelocity()
                : Constants.Spindexer.indexingMechanismRotationsPerSec);
      }
    }

    flywheel.periodic();
    spindexer.periodic();
    tunnel.periodic();

    /* TODO once these are all set up
    hood.periodic();
    turret.periodic();
    */
    Logger.recordOutput("Shooter/State", state.toString());
  }

  public boolean isRewindComplete() {
    return false; // TODO turret.isUnwound();
  }

  public boolean needsToUnwind() {
    return false; // TODO
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

  public boolean isTurretInPosition() {
    return false; // TODO
  }

  public void setState(ShooterState newState) {
    state = newState;
  }
}
