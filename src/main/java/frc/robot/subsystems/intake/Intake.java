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
  private IntakeState state = IntakeState.STARING_CONFIG;
  private IntakeState prevState = IntakeState.STARING_CONFIG;
  private boolean hasExtended = false;
  private boolean autoSmoosh = false;
  private boolean smooshingIn = true;

  public Intake(Deployer deployer, Rollers rollers) {
    this.deployer = deployer;
    this.rollers = rollers;
  }

  public enum IntakeState {
    STARING_CONFIG,
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

    if (DriverStation.isDisabled() && state != IntakeState.STARING_CONFIG) {
      setState(IntakeState.IDLE);
    }

    switch (state) {
      case STARING_CONFIG -> {
        deployer.setState(DeployerState.DISABLED);
        rollers.setState(RollersState.DISABLED);
        autoSmoosh = false;
      }
      case IDLE -> {
        if (!hasExtended) {
          deployer.setState(DeployerState.EXTEND);
          rollers.setState(RollersState.DEPLOY);
          if (deployer.isExtended()) {
            hasExtended = true;
          }
        } else {
          deployer.setState(DeployerState.EXTEND);
          rollers.setState(RollersState.IDLE);
        }
      }
      case INTAKING -> {
        if (!hasExtended) {
          deployer.setState(DeployerState.EXTEND);
          rollers.setState(RollersState.DEPLOY);
          if (deployer.isExtended()) {
            hasExtended = true;
          }
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
        if (autoSmoosh) {
          // If we ended auto while auto smooshing, return back to idle once teleop begins
          if (!DriverStation.isAutonomous()) {
            autoSmoosh = false;
            state = IntakeState.IDLE;
          }

          if (smooshingIn) {
            deployer.setState(DeployerState.SMOOSH);
          } else {
            deployer.setState(DeployerState.EXTEND);
          }

          if (deployer.isSmooshed() && smooshingIn) {
            smooshingIn = false;
          } else if (deployer.isExtended() && !smooshingIn) {
            smooshingIn = true;
          }

        } else {
          deployer.setState(DeployerState.SMOOSH);
        }
        rollers.setState(RollersState.SMOOSH);
      }
    }

    deployer.outputsPeriodic();
    rollers.outputsPeriodic();

    Logger.recordOutput("Intake/CurrentState", state);
    Logger.recordOutput("Intake/hasExtended", hasExtended);
    Logger.recordOutput("Intake/autoSmoosh", autoSmoosh);
    Logger.recordOutput("Intake/smooshingIn", smooshingIn);
  }

  public IntakeState getState() {
    return state;
  }

  public IntakeState getPrevState() {
    return prevState;
  }

  public void setState(IntakeState state) {
    Logger.recordOutput("Intake/RequestedState", state);
    prevState = this.state;
    this.state = state;
    smooshingIn = true;
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

  public void setAutoSmoosh(boolean enabled) {
    Logger.recordOutput("Intake/RequestedAutoSmoosh", enabled);
    autoSmoosh = enabled;
  }
}
