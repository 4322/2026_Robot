package frc.robot.subsystems.intake.deployer;

import java.util.function.*;

public class Deployer {
  private DeployerIO deployerIO;
  public double currentPosition;
  public double desiredPosition;

  public enum deployerGoal {
    DISABLED,
    EXTEND,
    RETRACT,
    UNJAM // TODO
  }

  public deployerGoal goal = deployerGoal.DISABLED;
  public deployerGoal prevGoal;

  public Deployer(DeployerIO deployerIO) {
    this.deployerIO = deployerIO;
  }

  public void periodic() {
    prevGoal = goal;
    switch (goal) {
      case DISABLED -> {}
      case EXTEND -> {
        extend();
      }
      case RETRACT -> {
        retract();
      }
    }
  }

  public void retract() {
    // deployerIO.set(Constants.retract)
  }

  public void extend() {
    // deployerIO.setPos(constants.Extend)
  }

  public void unjam() {
    // deployerIO.setPos() TODO
  }

  public void setBrakeMode(boolean mode) {
    // deployerIO.setBrakeMode(mode);
  }

  public Boolean isExtended() {
    // return (//TODO check votlage) ? true: false;
    return true;
  }

  public void setGoal(deployerGoal goal) {
    this.goal = goal;
  }
}
