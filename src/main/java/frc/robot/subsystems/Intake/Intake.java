package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Deployer.Deployer;
import frc.robot.subsystems.Intake.Rollers.Rollers;

public class Intake extends SubsystemBase {
  private final Deployer deployer;
  private final Rollers rollers;

  public Intake(Deployer deployer, Rollers rollers) {
    this.deployer = deployer;
    this.rollers = rollers;
  }

  public enum Goal {
    Disabled,
    Extend,
    Retract,
    Eject,
    Idle,
    Intaking,
    UnjamIG
  }

  public Goal desiredGoal = Goal.Disabled;

  @Override
  public void periodic() {

    switch (desiredGoal) {
      case Disabled:
        break;

      default:
        break;
    }
  }

  public void setGoal(Goal desiredGoal) {
    this.desiredGoal = desiredGoal;
  }
}
