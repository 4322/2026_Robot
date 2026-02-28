package frc.robot.subsystems.intake.deployer;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class DeployerIOTalonFX implements DeployerIO {
  private TalonFX deployerMotor;
  private CANcoder canCoder;
  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
  private CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
  public double requestedPosDeg;
  private double posRot;

  public DeployerIOTalonFX() {
    deployerMotor = new TalonFX(Constants.Deployer.motorId, Constants.CANivore.CANBus);
    canCoder = new CANcoder(Constants.Deployer.CANCoderID, Constants.CANivore.CANBus);

    motorConfigs.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();

    motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfigs.Feedback.SensorToMechanismRatio = Constants.Deployer.sensorToMechanismRatio;
    motorConfigs.Feedback.RotorToSensorRatio = Constants.Deployer.RotorToSensorRatio;
    motorConfigs.CurrentLimits.StatorCurrentLimit = Constants.Deployer.statorCurrentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = Constants.Deployer.supplyCurrentLimit;
    motorConfigs.MotorOutput.Inverted = Constants.Deployer.motorInvert;
    motorConfigs.MotorOutput.NeutralMode = Constants.Deployer.neutralMode;

    motorConfigs.Slot0.kP = Constants.Deployer.kP;
    motorConfigs.Slot0.kI = Constants.Deployer.kI;
    motorConfigs.Slot0.kD = Constants.Deployer.kD;
    motorConfigs.Slot0.kG = Constants.Deployer.kG;

    motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;
    StatusCode deployerConfigStatus = deployerMotor.getConfigurator().apply(motorConfigs);
    StatusCode CANcoderStatus = canCoder.getConfigurator().apply(canCoderConfigs);
    if (deployerConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + deployerMotor.getDeviceID()
              + " error (Deployer): "
              + deployerConfigStatus.getDescription(),
          false);
    }
    if (CANcoderStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + canCoder.getDeviceID()
              + " error (CANCoder): "
              + CANcoderStatus.getDescription(),
          false);
    }
    posRot =
        canCoder.getAbsolutePosition().getValueAsDouble()
            - Units.rotationsToDegrees(Constants.Deployer.CANCoderStowed);
    deployerMotor.setPosition(
        Units.degreesToRotations(Constants.Deployer.maxGravityDegrees) - posRot);
  }

  @Override
  public void updateInputs(DeployerIOInputs inputs) {
    // Implementation for updating inputs from the TalonFX hardware

    inputs.connected = deployerMotor.isConnected();

    inputs.angleDeg =
        toCodeCoords(Units.rotationsToDegrees(deployerMotor.getPosition().getValueAsDouble()));

    inputs.requestedPosDeg = requestedPosDeg;

    inputs.motorRotationsPerSec =
        Units.degreesToRotations(deployerMotor.getVelocity().getValueAsDouble());

    inputs.busCurrentAmps = deployerMotor.getSupplyCurrent().getValueAsDouble();

    inputs.statorCurrentAmps = deployerMotor.getStatorCurrent().getValueAsDouble();

    inputs.motorTempCelcius = deployerMotor.getDeviceTemp().getValueAsDouble();

    inputs.appliedVolts = deployerMotor.getMotorVoltage().getValueAsDouble();

    inputs.encoderRotations = canCoder.getAbsolutePosition().getValueAsDouble();

    inputs.motorRotations = deployerMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setPosition(double requestedPosDeg) {
    this.requestedPosDeg = requestedPosDeg;
    deployerMotor.setControl(
        new MotionMagicVoltage(Units.degreesToRotations(toMotorCoords(requestedPosDeg)))
            .withSlot(0)
            .withEnableFOC(true));
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
  public void enableBrakeMode(boolean mode) {
    deployerMotor.setNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  private double toCodeCoords(double position) {
    return position + Constants.Deployer.maxGravityDegrees;
  }

  private double toMotorCoords(double position) {
    return position - Constants.Deployer.maxGravityDegrees;
  }
}
