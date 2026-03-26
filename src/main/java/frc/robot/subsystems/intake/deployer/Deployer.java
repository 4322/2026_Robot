package frc.robot.subsystems.intake.deployer;

import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import org.littletonrobotics.junction.Logger;

public class Deployer {
  private DeployerIO deployerIO;
  private DeployerIOInputsAutoLogged inputs = new DeployerIOInputsAutoLogged();
  private double requestedPos;

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
    Logger.processInputs("Intake/Deployer", inputs);
  }

  // Called at end of command processing in intake
  public void outputsPeriodic() {
    Logger.recordOutput("Intake/Deployer/isSmooshed", isSmooshed());
    Logger.recordOutput("Intake/Deployer/isExtended", isExtended());
    Logger.recordOutput("Intake/Deployer/isStowed", isStowed());
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

  public boolean isSmooshed() {
    if (Constants.deployerMode == SubsystemMode.DISABLED) {
      return true;
    } else {
      return inputs.angleDeg >= Constants.Deployer.smooshDeg - Constants.Deployer.tolerance;
    }
  }

  public void setState(DeployerState state) {
    switch (state) {
      case DISABLED -> {
        break;
      }
      case EXTEND -> {
        requestedPos = Constants.Deployer.extendDeg;
        deployerIO.setPosition(requestedPos);
      }
      case SMOOSH -> {
        requestedPos = Constants.Deployer.smooshDeg;
        deployerIO.setPosition(requestedPos);
      }
    }
    Logger.recordOutput("Intake/Deployer/state", state);
    Logger.recordOutput("Intake/Deployer/requestedPos", requestedPos);
  }

  public boolean isStowed() {
    if (Constants.deployerMode == SubsystemMode.DISABLED) {
      return true;
    } else {
      return inputs.angleDeg <= Constants.Deployer.retractDeg + Constants.Deployer.tolerance;
    }
  }

  public double getAngle() {
    return inputs.angleDeg;
  }

  public void seedPosition(double newAngleDeg) {
    deployerIO.seedPosition(newAngleDeg);
  }
}
