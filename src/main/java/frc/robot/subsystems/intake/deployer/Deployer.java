package frc.robot.subsystems.intake.deployer;

import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import org.littletonrobotics.junction.Logger;

public class Deployer {
  private DeployerIO deployerIO;
  private DeployerIOInputsAutoLogged inputs = new DeployerIOInputsAutoLogged();

  public enum DeployerState {
    DISABLED,
    EXTEND,
    SMOOSH
  }

  public Deployer(DeployerIO deployerIO) {
    this.deployerIO = deployerIO;
  }

  public void inputsPeriodic() {
    deployerIO.updateInputs(inputs);
    Logger.processInputs("Deployer", inputs);
  }

  // Called at end of command processing in intake
  public void outputsPeriodic() {
    // Nothing here currently
  }

  public void setBrakeMode(boolean mode) {
    deployerIO.setBrakeMode(mode);
  }

  public boolean isExtended() {
    if (Constants.deployerMode == SubsystemMode.DISABLED) {
      return true;
    } else {
      return inputs.angleDeg >= Constants.Deployer.extendDeg - Constants.Deployer.tolerance;
    }
  }

  public void setState(DeployerState state) {
    Logger.recordOutput("Deployer/state", state);
    switch (state) {
      case DISABLED -> {
        break;
      }
      case EXTEND -> {
        deployerIO.setPosition(Constants.Deployer.extendDeg);
      }
      case SMOOSH -> {
        deployerIO.setPosition(Constants.Deployer.smooshDeg);
      }
    }
  }

  public boolean isStowed() {
    if (Constants.deployerMode == SubsystemMode.DISABLED) {
      return true;
    } else {
      return inputs.angleDeg <= Constants.Deployer.retractDeg + Constants.Deployer.tolerance;
    }
  }
}
