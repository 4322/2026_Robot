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
  private boolean wasExtended;

  public Intake(Deployer deployer, Rollers rollers) {
    this.deployer = deployer;
    this.rollers = rollers;
  }

  public enum IntakeState {
    DISABLED,
    RETRACT,
    EJECT,
    IDLE,
    INTAKING,
    // aadd unjamstill blue and not a priority in docs as of current so TODO
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/State", state);
    switch (state) {
      case DISABLED -> {
        break;
      }
      case RETRACT -> {
        deployer.setGoal(DeployerState.RETRACT);
        rollers.setState(RollersState.IDLE);
      }
      case EJECT -> {
        rollers.setState(RollersState.EJECT);
      }
      case IDLE -> {
        if (deployer.isExtended()) {
          wasExtended = true;
          rollers.setState(RollersState.IDLE);
        } else {
          deployer.setGoal(DeployerState.EXTEND);
          if (!wasExtended) {
            // untangle from the net
            rollers.setState(RollersState.DEPLOY);
          }
        }
      }
      case INTAKING -> {
        if (deployer.isExtended()) {
          wasExtended = true;
          rollers.setState(RollersState.INTAKE);
        } else {
          deployer.setGoal(DeployerState.EXTEND);
          if (!wasExtended) {
            // untangle from the net
            rollers.setState(RollersState.DEPLOY);
          }
        }
      }
    }
    deployer.periodic();
    rollers.periodic();
  }

  public void setState(IntakeState desiredState) {
    state = desiredState;
  }

  public IntakeState getState() {
    return state;
  }

  public boolean isExtended() {
    return deployer.isExtended();
  }

  public void enableBrakeMode(boolean enable) {
    deployer.setBrakeMode(enable);
  }
}
