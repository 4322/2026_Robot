package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
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
        if (DriverStation.isEnabled()) {
          prevState = state;
          state = IntakeState.DEPLOY;
        }
      }
      case DEPLOY -> {
        deployer.setState(DeployerState.EXTEND);
        rollers.setState(RollersState.DEPLOY);
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

    deployer.periodic();
    rollers.periodic();
  }

  public IntakeState getState() {
    return state;
  }

  public IntakeState getPrevState() {
    return prevState;
  }

  public void setState(IntakeState state) {
    prevState = this.state;
    this.state = state;
  }

  public boolean isExtended() {
    return deployer.isExtended();
  }

  public void setBrakeMode(boolean enable) {
    deployer.setBrakeMode(enable);
    rollers.setBrakeMode(enable);
  }
}
