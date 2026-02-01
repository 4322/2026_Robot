package frc.robot.subsystems.intake.deployer;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Deployer {
  private DeployerIO deployerIO;
  public double currentPosition;
  public double desiredPosition;
  private DeployerIOInputsAutoLogged inputs = new DeployerIOInputsAutoLogged();

  public enum deployerGoal {
    DISABLED,
    EXTEND,
    RETRACT,
    // UNJAM TODO
  }

  public deployerGoal goal = deployerGoal.DISABLED;

  public Deployer(DeployerIO deployerIO) {
    this.deployerIO = deployerIO;
  }

  public void periodic() {
    deployerIO.updateInputs(inputs);
    Logger.processInputs("Deployer", inputs);
    Logger.recordOutput("Deployer/Goal", goal);
    if (desiredPosition == inputs.angleDeg) {
      currentPosition = desiredPosition;
    }
    switch (Constants.deployerMode) {
      case DISABLED:
        break;
      case TUNING:
        break;
      case DRIVETUNING:
        break;
      case NORMAL:
        switch (goal) {
          case DISABLED -> {
            if (DriverStation.isEnabled()) {
              goal = deployerGoal.EXTEND;
            }
            break;
          }
          case EXTEND -> {
            extend();
          }
          case RETRACT -> {
            retract();
          }
        }
    }
  }

  public void retract() {
    desiredPosition = Constants.Deployer.retractDeg;
    deployerIO.setPosition(Constants.Deployer.retractDeg);
  }

  public void extend() {
    deployerIO.setPosition(Constants.Deployer.extendDeg);
    currentPosition = Constants.Deployer.extendDeg;
  }

  public void unjam() {
    // TODO
  }

  public void setBrakeMode(boolean mode) {
    deployerIO.enableBrakeMode(mode);
  }

  public Boolean isExtended() {
    if ((inputs.angleDeg <= Constants.Deployer.extendDeg)
        || (Constants.Deployer.extendDeg >= Math.abs(inputs.angleDeg - 0.01))) {
      return true;
    } else {
      return false;
    }
  }

  public void setGoal(deployerGoal goal) {
    this.goal = goal;
  }

  public boolean isStowed() {
    if ((inputs.angleDeg - 0.01 <= Constants.Deployer.retractDeg)
        || (Constants.Deployer.retractDeg >= Math.abs(inputs.angleDeg))) {
      return true;
    } else {
      return false;
    }
  }
}
