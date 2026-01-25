package frc.robot.subsystems.intake.rollers;

public class Rollers {
  public enum rollerSGoal {
    DISABLED,
    IDLE,
    INTAKE,
    EJECT
  }

  public rollerSGoal goal = rollerSGoal.DISABLED;
  public rollerSGoal prevGoal;

  public void periodic() {}

  public void setBrakeMode(boolean mode) {}

  public void setGoal(rollerSGoal goal) {
    this.goal = goal;
  }
}
