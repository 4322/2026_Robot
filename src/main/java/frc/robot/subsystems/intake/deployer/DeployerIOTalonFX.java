package frc.robot.subsystems.intake.deployer;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;

public class DeployerIOTalonFX implements DeployerIO {
  private TalonFX deployerMotor;
  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
  public double requestedPosDegosDeg;

  public DeployerIOTalonFX() {
    deployerMotor = new TalonFX(1);
    // Setup config objects

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;

    StatusCode deployerConfigStatus = deployerMotor.getConfigurator().apply(motorConfigs);


    if (deployerConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + deployerMotor.getDeviceID()
              + " error (Right Deployer): "
              + deployerConfigStatus.getDescription(),
          false);
    }
  }

  @Override
  public void updateInputs(DeployerIOInputs inputs) {
    // Implementation for updating inputs from the TalonFX hardware

    inputs.connected = deployerMotor.isConnected();

    inputs.requestedPosDeg = deployerMotor.getPosition().getValueAsDouble();

    inputs.requestedPosDeg = degreesToRotations(deployerMotor.getPosition().getValueAsDouble());

    inputs.speedRotationsPerSec =
        degreesToRotations(deployerMotor.getVelocity().getValueAsDouble());

    inputs.busCurrentAmps = deployerMotor.getSupplyCurrent().getValueAsDouble();

    inputs.statorCurrentAmps = deployerMotor.getStatorCurrent().getValueAsDouble();

    inputs.motorTempCelcius = deployerMotor.getDeviceTemp().getValueAsDouble();

    inputs.appliedVolts = deployerMotor.getMotorVoltage().getValueAsDouble();
    inputs.encoderRotations = deployerMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setPosition(double DeployerPositionMeters) {}

  @Override
  public void setVoltage(double voltage) {
    deployerMotor.setVoltage(voltage);
  }

  @Override
  public void stop() {
    deployerMotor.stopMotor();
  }

  @Override
  public TalonFX getTalonFX() {
    return deployerMotor;
  }

  @Override
  public void enableBrakeMode(Boolean mode) {
    deployerMotor.setNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  public double degreesToRotations(double value) {
    // TODO
    return value;
  }
}
