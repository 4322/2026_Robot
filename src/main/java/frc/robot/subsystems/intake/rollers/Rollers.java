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

  public double getFollowerSpeed() {
    return inputs.followerRotationsPerSec;
  }

  public double getLeaderSpeed() {
    return inputs.leaderRotationsPerSec;
  }

  public double getRequestedSetpoint() {
    return requestedSetpoint;
  }

  public boolean leaderAtGoal() {
    return Math.abs(inputs.leaderRotationsPerSec - requestedSetpoint) < 0.1;
  }

  public boolean followerAtGoal() {
    return Math.abs(inputs.followerRotationsPerSec - requestedSetpoint) < 0.1;
  }

  public boolean rollersSpinningTogether() {
    return MathUtil.isNear(inputs.leaderRotationsPerSec, inputs.followerRotationsPerSec, 0.01);
  }

  public boolean leaderConnected() {
    return inputs.leaderConnected;
  }

  public boolean followerConnected() {
    return inputs.followerConnected;
  }

  
  public boolean isCurrentConsistent() {
    return Math.abs(inputs.leaderSupplyCurrentAmps- inputs.followerSupplyCurrentAmps)
        < Constants.Flywheel.consistentCurrentToleranceAmps;
  }

  public double getFollowerCurrent(){
    return inputs.followerSupplyCurrentAmps;
  }

    public double getLeaderCurrent(){
    return inputs.leaderSupplyCurrentAmps;
  }
}
