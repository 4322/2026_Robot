package frc.robot.subsystems.intake.deployer;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
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

  public DeployerIOTalonFX() {
    deployerMotor = new TalonFX(Constants.Deployer.motorId, Constants.CANivore.CANBus);
    canCoder = new CANcoder(Constants.Deployer.CANCoderID, Constants.CANivore.CANBus);

    motorConfigs.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();

    // encoder is not absolute, only start in retracted position and can't use FusedCandoer!
    motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motorConfigs.Feedback.SensorToMechanismRatio =
        Constants.Deployer.sensorToMechanismRatio * Constants.Deployer.RotorToSensorRatio;
    motorConfigs.Feedback.RotorToSensorRatio = Constants.Deployer.RotorToSensorRatio;
    motorConfigs.CurrentLimits.StatorCurrentLimit = Constants.Deployer.statorCurrentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = Constants.Deployer.supplyCurrentLimit;
    motorConfigs.MotorOutput.Inverted = Constants.Deployer.motorInvert;
    motorConfigs.MotorOutput.NeutralMode = Constants.Deployer.neutralMode;

    motorConfigs.Slot0.kP = Constants.Deployer.kP;
    motorConfigs.Slot0.kI = Constants.Deployer.kI;
    motorConfigs.Slot0.kD = Constants.Deployer.kD;
    motorConfigs.Slot0.kG = Constants.Deployer.kG;
    motorConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motorConfigs.Slot0.GravityArmPositionOffset =
        -Units.degreesToRadians(Constants.Deployer.maxGravityDegrees);

    motorConfigs.MotionMagic.MotionMagicCruiseVelocity =
        Constants.Deployer.motionMagicCruiseVelocity;
    motorConfigs.MotionMagic.MotionMagicAcceleration = Constants.Deployer.motionMagicAcceleration;

    motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;
    motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;

    canCoderConfigs.MagnetSensor.MagnetOffset = -Constants.Deployer.SesnorOffsetRotations;
    canCoderConfigs.MagnetSensor.SensorDirection = Constants.Deployer.sensorDirection;
    canCoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1; // range 0 to 1.0

    // Must configure CANcoder before TalonFX since TalonFX is using CANcoder as feedback device
    StatusCode CANcoderStatus = canCoder.getConfigurator().apply(canCoderConfigs);
    if (CANcoderStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + canCoder.getDeviceID()
              + " error (CANCoder): "
              + CANcoderStatus.getDescription(),
          false);
    }
    StatusCode deployerConfigStatus = deployerMotor.getConfigurator().apply(motorConfigs);
    if (deployerConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + deployerMotor.getDeviceID()
              + " error (Deployer): "
              + deployerConfigStatus.getDescription(),
          false);
    }

    try {
      // wait for encoder position to be received
      Thread.sleep(100);
    } catch (InterruptedException e) {
    }
    deployerMotor.setPosition(
        canCoder.getAbsolutePosition().getValueAsDouble()
            / Constants.Deployer.sensorToMechanismRatio);
  }

  @Override
  public void updateInputs(DeployerIOInputs inputs) {
    // Implementation for updating inputs from the TalonFX hardware

    inputs.connected = deployerMotor.isConnected();

    inputs.angleDeg = Units.rotationsToDegrees(deployerMotor.getPosition().getValueAsDouble());

    inputs.requestedPosDeg = requestedPosDeg;

    inputs.motorDegreesPerSec =
        Units.rotationsToDegrees(deployerMotor.getVelocity().getValueAsDouble());

    inputs.busCurrentAmps = deployerMotor.getSupplyCurrent().getValueAsDouble();

    inputs.statorCurrentAmps = deployerMotor.getStatorCurrent().getValueAsDouble();

    inputs.motorTempCelcius = deployerMotor.getDeviceTemp().getValueAsDouble();

    inputs.appliedVolts = deployerMotor.getMotorVoltage().getValueAsDouble();

    inputs.encoderRotations = canCoder.getAbsolutePosition().getValueAsDouble();

    inputs.motorRotations = deployerMotor.getRotorPosition().getValueAsDouble();
  }

  @Override
  public void setPosition(double requestedPosDeg) {
    this.requestedPosDeg = requestedPosDeg;
    deployerMotor.setControl(
        new MotionMagicVoltage(Units.degreesToRotations(requestedPosDeg))
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
  public void setBrakeMode(boolean mode) {
    deployerMotor.setNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
