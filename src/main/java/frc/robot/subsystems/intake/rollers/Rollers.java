package frc.robot.subsystems.intake.rollers;

import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Rollers {
  private RollersIO rollersIO;
  private RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  public enum rollersGoal {
    IDLE,
    INTAKE,
    EJECT
  }

  public rollersGoal goal = rollersGoal.IDLE;

  public Rollers(RollersIO rollersIO) {
    this.rollersIO = rollersIO;
  }

  public void periodic() {
    rollersIO.updateInputs(inputs);
    Logger.processInputs("Rollers", inputs);
    Logger.recordOutput("Rollers/Goal", goal);
    switch (Constants.rollersMode) {
      case DISABLED:
        break;
      case NORMAL:
        switch (goal) {
          case IDLE -> {
            idle();
          }
          case INTAKE -> {
            intake();
          }
          case EJECT -> {
            eject();
          }
        }
    }
  }

  public void setBrakeMode(boolean mode) {
    rollersIO.enableBreakMode(mode);
  }

  public void intake() {
    rollersIO.setVoltage(Constants.Rollers.voltageIntake);
  }

  public void eject() {
    rollersIO.setVoltage(Constants.Rollers.voltageEject);
  }

  public void idle() {
    rollersIO.stopMotor();
  }

  public void setGoal(rollersGoal goal) {
    this.goal = goal;
  }
}
