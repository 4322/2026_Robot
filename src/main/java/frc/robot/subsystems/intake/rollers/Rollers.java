package frc.robot.subsystems.intake.rollers;

import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Rollers {
  private RollersIO rollersIO;
  private RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  public enum RollersState {
    IDLE,
    DEPLOY,
    INTAKE,
    EJECT,
    SMOOSH,
    DISABLED
  }

  public RollersState state = RollersState.DISABLED;

  public Rollers(RollersIO rollersIO) {
    this.rollersIO = rollersIO;
  }

  public void periodic() {
    rollersIO.updateInputs(inputs);
  }

  public void setBrakeMode(boolean enable) {
    rollersIO.enableBrakeMode(enable);
  }

  public void setState(RollersState state) {
    Logger.recordOutput("Rollers/State", state);
    switch (state) {
      case DISABLED -> {
        rollersIO.stopMotor();
      }
      case DEPLOY -> {
        rollersIO.setVoltage(Constants.Rollers.voltageDeploy);
      }
      case IDLE -> {
        rollersIO.stopMotor();
      }
      case INTAKE -> {
        rollersIO.setVoltage(Constants.Rollers.voltageIntake);
      }
      case EJECT -> {
        rollersIO.setVoltage(Constants.Rollers.voltageEject);
      }
      case SMOOSH -> {
        rollersIO.setVoltage(Constants.Rollers.voltageSmoosh);
      }
    }
  }
}
