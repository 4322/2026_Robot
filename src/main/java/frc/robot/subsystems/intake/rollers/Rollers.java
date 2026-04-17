package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Rollers {
  private RollersIO rollersIO;
  private RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();
  private double requestedSetpoint = 0;

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

  public void inputsPeriodic() {
    rollersIO.updateInputs(inputs);
    Logger.processInputs("Intake/Rollers", inputs);
  }

  // Called at end of command processing in intake
  public void outputsPeriodic() {
    // Nothing here currently
  }

  public void setState(RollersState state) {
    Logger.recordOutput("Intake/Rollers/State", state);
    switch (state) {
      case DISABLED -> {
        rollersIO.stopMotor();
      }
      case DEPLOY -> {
        requestedSetpoint = Constants.Rollers.voltageDeploy;
        rollersIO.setVoltage(Constants.Rollers.voltageDeploy);
      }
      case IDLE -> {
        requestedSetpoint = 0;
        rollersIO.stopMotor();
      }
      case INTAKE -> {
        requestedSetpoint = Constants.Rollers.voltageIntake;
        rollersIO.setVoltage(Constants.Rollers.voltageIntake);
      }
      case EJECT -> {
        requestedSetpoint = Constants.Rollers.voltageEject;
        rollersIO.setVoltage(Constants.Rollers.voltageEject);
      }
      case SMOOSH -> {
        requestedSetpoint = Constants.Rollers.voltageSmoosh;
        rollersIO.setVoltage(Constants.Rollers.voltageSmoosh);
      }
    }
  }

  public double getFollowerRollerSpeed() {
    return inputs.followerRotationsPerSec;
  }

  public double getLeaderRollerSpeed() {
    return inputs.leaderRotationsPerSec;
  }

  public double getRequestedSetpoint() {
    return requestedSetpoint;
  }

  public boolean leaderRollerAtGoal() {
    return Math.abs(inputs.leaderRotationsPerSec - requestedSetpoint) < 0.1;
  }

  public boolean followerRollerAtGoal() {
    return Math.abs(inputs.followerRotationsPerSec - requestedSetpoint) < 0.1;
  }

  public boolean rollersSpinningTogether() {
    return MathUtil.isNear(inputs.leaderRotationsPerSec, inputs.followerRotationsPerSec, 0.01);
  }

  public boolean leaderRollerConnected() {
    return inputs.leaderConnected;
  }

  public boolean followerRollerConnected() {
    return inputs.followerConnected;
  }
}
