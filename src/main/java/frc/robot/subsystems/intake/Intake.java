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

  public Intake(Deployer deployer, Rollers rollers) {
    this.deployer = deployer;
    this.rollers = rollers;
  }

  public enum IntakeState {
    DISABLED,
    EXTEND,
    RETRACT,
    EJECT,
    IDLE,
    INTAKING,
    // aadd unjamstill blue and not a priority in docs as of current so TODO
  }

  public IntakeState state = IntakeState.DISABLED;
  public IntakeState prevState;

  @Override
  public void periodic() {
    Logger.recordOutput("Inttake/State", state);
    switch (state) {
      case DISABLED -> {
        break;
      }
      case EXTEND -> {
        deployer.setState(DeployerState.EXTEND);
        rollers.setState(RollersState.IDLE);
      }
      case RETRACT -> {
        deployer.setState(DeployerState.RETRACT);
        rollers.setState(RollersState.IDLE);
      }
      case EJECT -> {
        deployer.setState(DeployerState.EXTEND);
        rollers.setState(RollersState.EJECT);
      }
      case IDLE -> {
        deployer.setState(DeployerState.EXTEND);
        rollers.setState(RollersState.IDLE);
      }
      case INTAKING -> {
        deployer.setState(DeployerState.EXTEND);
        rollers.setState(RollersState.INTAKE);
      }
    }
    deployer.periodic();
    rollers.periodic();
  }

  public void setState(IntakeState desiredState) {
    prevState = state;
    state = desiredState;
  }

  public IntakeState getState() {
    return state;
  }

  public void enableBrakeMode(boolean enable) {
    deployer.setBrakeMode(enable);
    rollers.setBrakeMode(enable);
  }
}
