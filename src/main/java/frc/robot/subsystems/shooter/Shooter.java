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
    PRESHOOT, // Flywheel gets up to speed; Turret/hood aim
    SHOOT, // Spindexer and tunnel get up to speed
  }

  private ShooterState state = ShooterState.DISABLED;

  private Flywheel flywheel;
  private Hood hood;
  private Spindexer spindexer;
  private Tunnel tunnel;
  private Turret turret;

  private double targetAngle;
  private double targetFlywheelSpeed;

  private boolean unwindComplete = false;

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
          flywheel.requestGoal(Constants.Flywheel.idleMechanismRPS);
        }
      }
      case UNWIND -> {
        spindexer.requestIdle();
        if (spindexer.isStopped()) {
          tunnel.requestIdle();
          if (tunnel.isStopped()) {
            turret.preemptiveUnwind();
          }
        } // TODO hub enable check
        // TODO unwindComplete = turret.isUnwound();
      }
      case PRESHOOT -> {
        flywheel.requestGoal(targetFlywheelSpeed);
        // TODO Turret request position here and hood
      }
      case SHOOT -> {
        calculateFiringSolution();
        flywheel.requestGoal(targetFlywheelSpeed); // TODO change to variable
        // TODO Turret request position here and hood
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

  private void calculateFiringSolution() {
    // TODO will use firing solution manager here
    targetAngle = 0;
    targetFlywheelSpeed = 0;
  }

  public boolean isUnwindComplete() {
    return unwindComplete;
  }

  // Turret is at maximum path of travel
  public boolean needsToUnwind() {
    return false; // TODO
  }

  // Turret is at center position
  public boolean isUnwinded() {
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
    if (!(newState == state)) {
      state = newState;
      unwindComplete = false;
    }
  }

  public ShooterState getState() {
    return state;
  }
}
