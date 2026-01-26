package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.deployer.Deployer;
import frc.robot.subsystems.intake.deployer.Deployer.deployerGoal;
import frc.robot.subsystems.intake.rollers.Rollers;
import frc.robot.subsystems.intake.rollers.Rollers.rollersGoal;

public class Intake extends SubsystemBase {
  private final Deployer deployer;
  private final Rollers rollers;

  public Intake(Deployer deployer, Rollers rollers) {
    this.deployer = deployer;
    this.rollers = rollers;
  }

  public enum Goal {
    DISABLED,
    EXTEND,
    RETRACT,
    EJECT,
    IDLE,
    INTAKING,
    UNJAM // still blue and not a priority in docs as of current so TODO
  }

  public Goal goal = Goal.DISABLED;
  public Goal prevGoal;

  @Override
  public void periodic() {
    prevGoal = goal;
    switch (goal) {
      case EXTEND -> {
        deployer.setGoal(deployerGoal.EXTEND);
      }
      case RETRACT -> {
        deployer.setGoal(deployerGoal.RETRACT);
        rollers.setGoal(rollersGoal.IDLE);
      }
      case EJECT -> {
        deployer.setGoal(deployerGoal.EXTEND);
        rollers.setGoal(rollersGoal.EJECT);
      }
      case IDLE -> {
        // TODO logic to check hopper and switch between rollers goal
        deployer.setGoal(deployerGoal.EXTEND);
        rollers.setGoal(rollersGoal.IDLE);
      }
      case INTAKING -> {
        deployer.setGoal(deployerGoal.EXTEND);
        rollers.setGoal(rollersGoal.INTAKE);
      }
      case UNJAM -> { // TODO
      }
    }
    deployer.periodic();
    rollers.periodic();
    if (goal == Goal.EXTEND && !deployer.isExtended()) {
      setUNJAM();
    } else if (goal == Goal.EXTEND && deployer.isExtended()) {
      goal = Goal.IDLE;
    }
  }

  public void setGoal(Goal desiredGoal) {
    this.goal = desiredGoal;
  }

  public Goal getGoal() {
    return prevGoal;
  }

  public Goal setUNJAM() {
    return goal = (!deployer.isExtended()) ? Goal.UNJAM : goal;
  }
}
