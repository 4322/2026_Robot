package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.deployer.Deployer;
import frc.robot.subsystems.intake.deployer.Deployer.DeployerState;
import frc.robot.subsystems.intake.rollers.Rollers;
import frc.robot.subsystems.intake.rollers.Rollers.RollersState;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final Deployer deployer;
  private final Rollers rollers;
  private IntakeState state = IntakeState.DISABLED;
  private IntakeState prevState = IntakeState.DISABLED;
  private boolean hasExtended = false;

  public Intake(Deployer deployer, Rollers rollers) {
    this.deployer = deployer;
    this.rollers = rollers;
  }

  public enum IntakeState {
    DISABLED,
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

    switch (state) {
      case DISABLED -> {
        deployer.setState(DeployerState.DISABLED);
        rollers.setState(RollersState.DISABLED);
        hasExtended = false;
      }
      case DEPLOY -> {
        deployer.setState(DeployerState.EXTEND);
        rollers.setState(RollersState.DEPLOY);
        if (deployer.isExtended()) {
          hasExtended = true;
          state = IntakeState.INTAKING;
        }
      }
      case IDLE -> {
        if (!hasExtended) {
          state = IntakeState.DEPLOY;
        } else {
          deployer.setState(DeployerState.EXTEND);
          rollers.setState(RollersState.IDLE);
        }
      }
      case INTAKING -> {
        if (!hasExtended) {
          state = IntakeState.DEPLOY;
        } else {
          deployer.setState(DeployerState.EXTEND);
          rollers.setState(RollersState.INTAKE);
        }
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

    Logger.recordOutput("Intake/State", state);
    Logger.recordOutput("Intake/hasExtended", hasExtended);
  }

  public IntakeState getState() {
    return state;
  }

  public IntakeState getPrevState() {
    return prevState;
  }

  public boolean isDisabled() {
    return state == IntakeState.DISABLED;
  }

  public void setState(IntakeState state) {
    Logger.recordOutput("Intake/setState", state);
    prevState = this.state;
    this.state = state;
  }

  public boolean isExtended() {
    return deployer.isExtended();
  }

  // Has extended from deployment; hasn't gotten stuck in net
  public boolean hasExtended() {
    return hasExtended;
  }

  public void setBrakeMode(boolean enable) {
    deployer.setBrakeMode(enable);
    rollers.setBrakeMode(enable);
  }
}
