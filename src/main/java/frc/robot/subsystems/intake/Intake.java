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

  private boolean requestIdle = false;
  private boolean requestIntake = false;
  private boolean requestEject = false;
  private boolean requestSmoosh = false;

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
    Logger.recordOutput("Intake/State", state);
    switch (state) {
      case DISABLED -> {
        deployer.setState(DeployerState.DISABLED);
        rollers.setState(RollersState.DISABLED);
        if (requestIdle || requestIntake || requestEject || requestSmoosh) {
          state = IntakeState.DEPLOY;
        }
      }
      case DEPLOY -> {
        deployer.setState(DeployerState.EXTEND);
        rollers.setState(RollersState.DEPLOY);

        if (deployer.isExtended()) {
          if (requestIntake) {
            state = IntakeState.INTAKING;
          } else if (requestEject) {
            state = IntakeState.EJECT;
          } else if (requestSmoosh) {
            state = IntakeState.SMOOSH;
          } else {
            state = IntakeState.IDLE;
          }
        }
      }
      case IDLE -> {
        deployer.setState(DeployerState.EXTEND);
        rollers.setState(RollersState.IDLE);

        if (requestIntake) {
          state = IntakeState.INTAKING;
        } else if (requestEject) {
          state = IntakeState.EJECT;
        } else if (requestSmoosh /* TODO && ball path not unjamming && outtake in shoot*/) {
          state = IntakeState.SMOOSH;
        }
      }
      case INTAKING -> {
        deployer.setState(DeployerState.EXTEND);
        rollers.setState(RollersState.INTAKE);

        if (requestIdle) {
          state = IntakeState.IDLE;
        } else if (requestEject) {
          state = IntakeState.EJECT;
        } else if (requestSmoosh) {
          state = IntakeState.SMOOSH;
        }
      }
      case EJECT -> {
        deployer.setState(DeployerState.EXTEND);
        rollers.setState(RollersState.EJECT);

        if (requestIdle) {
          state = IntakeState.IDLE;
        } else if (requestIntake) {
          state = IntakeState.INTAKING;
        } else if (requestSmoosh) {
          state = IntakeState.SMOOSH;
        }
      }
      case SMOOSH -> {
        deployer.setState(DeployerState.SMOOSH);
        rollers.setState(RollersState.IDLE); // TODO figure out if smoosh will cause issues with net

        if (requestIdle) {
          state = IntakeState.IDLE;
        } else if (requestIntake) {
          state = IntakeState.INTAKING;
        } else if (requestEject) {
          state = IntakeState.EJECT;
        }
      }
    }

    deployer.periodic();
    rollers.periodic();
  }

  private void unsetRequests() {
    requestIdle = false;
    requestIntake = false;
    requestEject = false;
    requestSmoosh = false;
  }

  public void requestIdle() {
    unsetRequests();
    requestIdle = true;
  }

  public void requestIntake() {
    unsetRequests();
    requestIntake = true;
  }

  public void requestEject() {
    unsetRequests();
    requestEject = true;
  }

  public void requestSmoosh() {
    unsetRequests();
    requestSmoosh = true;
  }

  public IntakeState getState() {
    return state;
  }

  public void setBrakeMode(boolean enable) {
    deployer.setBrakeMode(enable);
    rollers.setBrakeMode(enable);
  }
}
