package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Flywheel {
  private FlywheelIO io;
  private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private double requestedMechanismRPS = 0.0;

  public enum FlywheelStates {
    DISABLED,
    IDLE,
    SHOOTING
  }

  private FlywheelStates state = FlywheelStates.DISABLED;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    switch (state) {
      case DISABLED -> {
        if (DriverStation.isEnabled()) {
          state = FlywheelStates.IDLE;
        }
      }
      case IDLE -> {
        io.setTargetMechanismRotations(Constants.Flywheel.idleMechanismRPS);
      }
      case SHOOTING -> {
        io.setTargetMechanismRotations(requestedMechanismRPS);
      }
    }

    Logger.recordOutput("Flywheel/State", state.toString());
  }

  public void requestIdle() {
    state = FlywheelStates.IDLE;
  }

  public void requestShoot(double velocity) {
    state = FlywheelStates.SHOOTING;
    requestedMechanismRPS = velocity;
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }
}
