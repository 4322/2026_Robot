package frc.robot.subsystems.intake.deployer;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import org.littletonrobotics.junction.Logger;

public class Deployer {
  private DeployerIO deployerIO;
  private DeployerIOInputsAutoLogged inputs = new DeployerIOInputsAutoLogged();
  private double requestedPos;
  private Timer deployTimer = new Timer();
  private DeployerState deployerState = DeployerState.DISABLED;

  public enum DeployerState {
    DISABLED,
    FIRST_EXTEND,
    EXTEND,
    SMOOSH
  }

  public Deployer(DeployerIO deployerIO) {
    this.deployerIO = deployerIO;
  }

  public void inputsPeriodic() {
    deployerIO.updateInputs(inputs);
    Logger.processInputs("Intake/Deployer", inputs);
  }

  // Called at end of command processing in intake
  public void outputsPeriodic() {
    Logger.recordOutput("Intake/Deployer/isSmooshed", isSmooshed());
    Logger.recordOutput("Intake/Deployer/isExtended", isExtended());
    Logger.recordOutput("Intake/Deployer/isStowed", isStowed());
  }

  public void setDeployerState(DeployerState state) {
    deployerState = state;
    if (deployerState != DeployerState.FIRST_EXTEND) {
      deployTimer.stop();
      deployTimer.reset();
    }
    switch (deployerState) {
      case DISABLED -> {
        break;
      }
      case FIRST_EXTEND -> {
        deployerIO.setVoltage(Constants.Deployer.deployVoltage);
        deployTimer.start();
        if (deployTimer.hasElapsed(Constants.Deployer.deploySec)) {
          deployerIO.seedPosition(Constants.Deployer.pressedIntoBumperDeg);
          deployerIO.setVoltage(0);
          deployerState = DeployerState.EXTEND;
        }
      }
      case EXTEND -> {
        requestedPos = Constants.Deployer.extendDeg;
        if (isExtended()) {
          deployerIO.setVoltage(0); // drop to bumper in coast mode to avoid stalling motor
        } else {
          deployerIO.setPosition(requestedPos);
        }
      }
      case SMOOSH -> {
        requestedPos = Constants.Deployer.smooshDeg;
        deployerIO.setPosition(requestedPos);
      }
    }
    Logger.recordOutput("Intake/Deployer/state", deployerState);
    Logger.recordOutput("Intake/Deployer/requestedPos", requestedPos);
  }

  public boolean isStowed() {
    if (Constants.deployerMode == SubsystemMode.DISABLED) {
      return true;
    } else if (deployerState == DeployerState.DISABLED
        || deployerState == DeployerState.FIRST_EXTEND) {
      return false;
    } else {
      return inputs.angleDeg <= Constants.Deployer.retractDeg + Constants.Deployer.tolerance;
    }
  }

  public boolean isExtended() {
    if (Constants.deployerMode == SubsystemMode.DISABLED) {
      return true;
    } else if (deployerState == DeployerState.DISABLED
        || deployerState == DeployerState.FIRST_EXTEND) {
      return false;
    } else {
      return inputs.angleDeg >= Constants.Deployer.extendDeg - Constants.Deployer.tolerance;
    }
  }

  public boolean isSmooshed() {
    if (Constants.deployerMode == SubsystemMode.DISABLED) {
      return true;
    } else if (deployerState == DeployerState.DISABLED
        || deployerState == DeployerState.FIRST_EXTEND) {
      return false;
    } else {
      return inputs.angleDeg >= Constants.Deployer.smooshDeg - Constants.Deployer.tolerance
          && inputs.angleDeg <= Constants.Deployer.smooshDeg + Constants.Deployer.tolerance;
    }
  }

  public double getAngle() {
    return inputs.angleDeg;
  }
}
