package frc.robot.subsystems.intake.deployer;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Deployer {
  private DeployerIO deployerIO;
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
    switch (Constants.deployerMode) {
      case DISABLED -> {}
      case TUNING -> {}
      case DRIVE_TUNING -> {}
      case NORMAL -> {
        switch (goal) {
          case DISABLED -> {
            if (DriverStation.isEnabled()) {
              goal = deployerGoal.EXTEND;
            }
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
  }

  public void retract() {
    deployerIO.setPosition(Constants.Deployer.retractDeg);
  }

  public void extend() {
    deployerIO.setPosition(Constants.Deployer.extendDeg);
  }

  public void unjam() {
    // TODO
  }

  public void setBrakeMode(boolean mode) {
    deployerIO.enableBrakeMode(mode);
  }

  public Boolean isExtended() {
    return (inputs.angleDeg >= Constants.Deployer.extendDeg - Constants.Deployer.tolerance)
        ? true
        : false;
  }

  public void setGoal(deployerGoal goal) {
    this.goal = goal;
  }

  public boolean isStowed() {
    return (inputs.angleDeg <= Constants.Deployer.retractDeg + Constants.Deployer.tolerance)
        ? true
        : false;
  }
}
