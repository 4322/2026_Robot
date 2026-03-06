package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.deployer.Deployer;
import frc.robot.subsystems.intake.rollers.Rollers;

public class Intake extends SubsystemBase {
  private final Deployer deployer;
  private final Rollers rollers;

  public Intake(Deployer deployer, Rollers rollers) {
    this.deployer = deployer;
    this.rollers = rollers;
  }

  public enum IntakeState {
    DISABLED,
    RETRACT,
    EJECT,
    IDLE,
    INTAKING,
    // aadd unjamstill blue and not a priority in docs as of current so TODO
  }

  public IntakeState state = IntakeState.DISABLED;

  @Override
  public void periodic() {}

  public void setState(IntakeState desiredState) {
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
