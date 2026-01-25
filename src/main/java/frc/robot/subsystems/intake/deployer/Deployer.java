package frc.robot.subsystems.intake.deployer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.*;

public class Deployer extends SubsystemBase {
  private DeployerIO deployerIO;
  public double currentPosition;
  public double desiredPosition;

  public Deployer(DeployerIO deployerIO) {
    this.deployerIO = deployerIO;
  }

  public void periodic() {}

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
}
