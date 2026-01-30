package frc.robot.subsystems.shooter.tunnel;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class TunnelIOTalonFx implements TunnelIO {
  private TalonFX motor;
  private double lastRequestedVelocity = -1;

  private TalonFXConfiguration config = new TalonFXConfiguration();
  private VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public TunnelIOTalonFx() {
    motor = new TalonFX(Constants.Tunnel.tunnelMotorId);

    config.CurrentLimits.StatorCurrentLimit = Constants.Tunnel.statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Tunnel.supplyCurrentLimit;
    config.MotorOutput.Inverted = Constants.Tunnel.motorInvert;
    config.MotorOutput.NeutralMode = Constants.Tunnel.neutralMode;

    config.Slot0.kS = Constants.Tunnel.kS;
    config.Slot0.kV = Constants.Tunnel.kV;
    config.Slot0.kP = Constants.Tunnel.kP;
    config.Slot0.kI = Constants.Tunnel.kI;
    config.Slot0.kD = Constants.Tunnel.kD;

    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;

    StatusCode configStatus = motor.getConfigurator().apply(config);

    if (configStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon " + motor.getDeviceID() + " error (Tunnel): " + configStatus.getDescription(),
          false);
    }
  }

  @Override
  public void updateInputs(TunnelIOInputs inputs) {
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
              .withVelocity(velocity / Constants.Tunnel.motorToMechanismRatio)
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
