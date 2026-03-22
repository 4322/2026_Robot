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
    DISABLED
  }

  public RollersState state = RollersState.DISABLED;

  public Rollers(RollersIO rollersIO) {
    this.rollersIO = rollersIO;
  }

  public void periodic() {
    rollersIO.updateInputs(inputs);
    Logger.processInputs("Rollers", inputs);

    switch (Constants.rollerMode) {
      case TUNING -> {}
      case DISABLED -> {}
      case NORMAL -> {
        switch (state) {
          case DISABLED -> {}
          case DEPLOY -> {
            rollersIO.setVoltage(Constants.Rollers.voltageDeploy);
          }
          case IDLE -> {
            rollersIO.setVoltage(Constants.Rollers.voltageIdle);
          }
          case INTAKE -> {
            rollersIO.setVoltage(Constants.Rollers.voltageIntake);
          }
          case EJECT -> {
            rollersIO.setVoltage(Constants.Rollers.voltageEject);
          }
        }
        Logger.recordOutput("Rollers/state", state);
      }
    }
  }

  public void setBrakeMode(boolean enable) {
    rollersIO.enableBrakeMode(enable);
  }

  public void setState(RollersState newState) {
    state = newState;
  }
}
