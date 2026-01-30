package frc.robot.subsystems.shooter.spindexer;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class SpindexerIOTalonFx implements SpindexerIO {
  private TalonFX motor;
  private double lastRequestedVelocity = -1;

  private TalonFXConfiguration config = new TalonFXConfiguration();
  private VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public SpindexerIOTalonFx() {
    motor = new TalonFX(Constants.Spindexer.spindexerMotorId);

    config.CurrentLimits.StatorCurrentLimit = Constants.Spindexer.statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Spindexer.supplyCurrentLimit;

    config.MotorOutput.Inverted = Constants.Spindexer.motorInvert;
    config.MotorOutput.NeutralMode = Constants.Spindexer.neutralMode;

    config.Slot0.kS = Constants.Spindexer.kS;
    config.Slot0.kV = Constants.Spindexer.kV;
    config.Slot0.kP = Constants.Spindexer.kP;
    config.Slot0.kI = Constants.Spindexer.kI;
    config.Slot0.kD = Constants.Spindexer.kD;

    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;

    StatusCode configStatus = motor.getConfigurator().apply(config);

    if (configStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon " + motor.getDeviceID() + " error (Spindexer): " + configStatus.getDescription(),
          false);
    }
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.motorConnected = motor.isConnected();
    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.velocityRotationsPerSec = motor.getVelocity().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.motorTempC = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setTargetVelocity(double velocity) {
    if (velocity != lastRequestedVelocity) {
      motor.setControl(
          velocityRequest
              .withVelocity(velocity / Constants.Spindexer.motorToMechanismRatio)
              .withEnableFOC(true));
    }

    lastRequestedVelocity = velocity;
  }

  @Override
  public void stop() {
    lastRequestedVelocity = 0;
    motor.setControl(velocityRequest.withVelocity(0).withEnableFOC(true));
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    motor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
