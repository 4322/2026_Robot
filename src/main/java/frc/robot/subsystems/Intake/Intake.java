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
//TODO remove this < comment so i can commit it  >remove this
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
