package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.deployer.Deployer;
import frc.robot.subsystems.intake.deployer.Deployer.deployerGoal;
import frc.robot.subsystems.intake.rollers.Rollers;
import frc.robot.subsystems.intake.rollers.Rollers.rollersGoal;
import org.littletonrobotics.junction.Logger;

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
    // aadd unjamstill blue and not a priority in docs as of current so TODO
  }

  public Goal goal = Goal.DISABLED;
  public Goal prevGoal;

  @Override
  public void periodic() {
    Logger.recordOutput("Inttake/State", goal);
    prevGoal = goal;
    switch (goal) {
      case DISABLED -> {
        break;
      }
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
        deployer.setGoal(deployerGoal.EXTEND);
        rollers.setGoal(rollersGoal.IDLE);
      }
      case INTAKING -> {
        deployer.setGoal(deployerGoal.EXTEND);
        rollers.setGoal(rollersGoal.INTAKE);
      }
    }
    deployer.periodic();
    rollers.periodic();
  }

  public void setGoal(Goal desiredGoal) {
    this.goal = desiredGoal;
  }

  public Goal getGoal() {
    return prevGoal;
  }

  public void enableBreakMode() {
    deployer.setBrakeMode(true);
    rollers.setBrakeMode(true);
  }
}
