package frc.robot.subsystems.intake.deployer;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;
import com.ctre.phoenix6.hardware.CANcoder;
public class DeployerIOTalonFX implements DeployerIO {
  private TalonFX deployerMotor;
  private CANcoder canCoder;
  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
  private CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
  public double requestedPosDegosDeg;

  public DeployerIOTalonFX() {
    deployerMotor = new TalonFX(Constants.Deployer.motorId);
    canCoder = new CANcoder(0);
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;

    StatusCode deployerConfigStatus = deployerMotor.getConfigurator().apply(motorConfigs);
    canCoder.getConfigurator().apply(canCoderConfigs);
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

    inputs.angleDeg = rotationsToDegrees(deployerMotor.getPosition().getValueAsDouble());

    inputs.requestedPosDeg = rotationsToDegrees(deployerMotor.getPosition().getValueAsDouble());

    inputs.speedRotationsPerSec =
        degreesToRotations(deployerMotor.getVelocity().getValueAsDouble());

    inputs.busCurrentAmps = deployerMotor.getSupplyCurrent().getValueAsDouble();

    inputs.statorCurrentAmps = deployerMotor.getStatorCurrent().getValueAsDouble();

    inputs.motorTempCelcius = deployerMotor.getDeviceTemp().getValueAsDouble();

    inputs.appliedVolts = deployerMotor.getMotorVoltage().getValueAsDouble();
    
    inputs.encoderRotations = canCoder.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public void setPosition(double requestedPosDeg) {
    int slot = (Deployer.prevGoal == Deployer.deployerGoal.EXTEND) ? 0 : 1;
    deployerMotor.setControl(
        new MotionMagicVoltage(Units.degreesToRotations(requestedPosDeg))
            .withSlot(slot)
            .withEnableFOC(true));
  }

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
  public double rotationsToDegrees(double value){
    return value;//TODO
  }
}
