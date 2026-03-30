package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.deployer.Deployer;
import frc.robot.subsystems.intake.deployer.Deployer.DeployerState;
import frc.robot.subsystems.intake.rollers.Rollers;
import frc.robot.subsystems.intake.rollers.Rollers.RollersState;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final Deployer deployer;
  private final Rollers rollers;
  private IntakeState statee = IntakeState.STARING_CONFIG;
  private IntakeState prevState = IntakeState.STARING_CONFIG;
  private boolean hasExtended = false;
  private Timer deployCheckTimer = new Timer();
  private double initialDeployerAngle = 0;
  private boolean alreadyDeployedCheckFailed = false;

  public Intake(Deployer deployer, Rollers rollers) {
    this.deployer = deployer;
    this.rollers = rollers;
  }

  public enum IntakeState {
    STARING_CONFIG,
    DEPLOY,
    IDLE,
    EJECT,
    SMOOSH,
    INTAKING
  }

  @Override
  public void periodic() {
    rollers.inputsPeriodic();
    deployer.inputsPeriodic();
  }

  public void periodicOutputs() {

    if (DriverStation.isDisabled() && statee != IntakeState.STARING_CONFIG) {
      setState(IntakeState.IDLE);
    }

    switch (statee) {
      case STARING_CONFIG -> {
        deployer.setState(DeployerState.DISABLED);
        rollers.setState(RollersState.DISABLED);
      }
      case DEPLOY -> {
        deployer.setState(DeployerState.EXTEND);
        rollers.setState(RollersState.DEPLOY);

        if (!alreadyDeployedCheckFailed) {
          if (!deployCheckTimer.isRunning()) {
            initialDeployerAngle = deployer.getAngle();
            deployCheckTimer.start();
          }

          if (deployCheckTimer.hasElapsed(1)) {
            double currentDeployerAngle = deployer.getAngle();
            // Deployer is up against bumper and isn't moving
            if (currentDeployerAngle < Constants.Deployer.alreadyDeployedMaxDeg
                && Math.abs(initialDeployerAngle - currentDeployerAngle)
                    < Constants.Deployer.alreadyDeployedMoveThreshold) {
              // Add one sensor rotation to current reported position
              deployer.seedPosition(
                  currentDeployerAngle + (360 / Constants.Deployer.sensorToMechanismRatio));
              hasExtended = true;
              statee = prevState;
            } else {
              alreadyDeployedCheckFailed = true;
            }
            deployCheckTimer.stop();
            deployCheckTimer.reset();
          }
        }

        if (deployer.isExtended()) {
          hasExtended = true;
          statee = prevState;
          alreadyDeployedCheckFailed = true;
          deployCheckTimer.stop();
          deployCheckTimer.reset();
        }
      }
      case IDLE -> {
        deployer.setState(DeployerState.EXTEND);
        rollers.setState(RollersState.IDLE);
      }
      case INTAKING -> {
        deployer.setState(DeployerState.EXTEND);
        rollers.setState(RollersState.INTAKE);
      }
      case EJECT -> {
        deployer.setState(DeployerState.EXTEND);
        rollers.setState(RollersState.EJECT);
      }
      case SMOOSH -> {
        deployer.setState(DeployerState.SMOOSH);
        rollers.setState(RollersState.SMOOSH);
      }
    }

    deployer.outputsPeriodic();
    rollers.outputsPeriodic();

    Logger.recordOutput("Intake/CurrentState", statee);
    Logger.recordOutput("Intake/hasExtended", hasExtended);
    Logger.recordOutput("Intake/alreadyDeployCheckFailed", alreadyDeployedCheckFailed);
  }

  public IntakeState getState() {
    return statee;
  }

  public IntakeState getPrevState() {
    return prevState;
  }

  public void setState(IntakeState statee) {
    Logger.recordOutput("Intake/RequestedState", statee);
    if (hasExtended) {
      prevState = this.statee;
      this.statee = statee;
    } else {
      prevState = statee;
      this.statee = IntakeState.DEPLOY;
    }
  }

  public boolean isExtended() {
    return deployer.isExtended();
  }

  public boolean isSmooshed() {
    return deployer.isSmooshed();
  }

  // Has extended from deployment; hasn't gotten stuck in net
  public boolean hasExtended() {
    return hasExtended;
  }
}
