package frc.robot.subsystems.intake.deployer;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class DeployerIOTalonFX implements DeployerIO {
  private TalonFX deployerMotor;
  private CANcoder canCoder;
  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
  private CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
  private Slot0Configs pidConfig = new Slot0Configs();
  public double requestedPosDeg;
  private double posRotError;

  public DeployerIOTalonFX() {
    deployerMotor = new TalonFX(Constants.Deployer.motorId);
    canCoder = new CANcoder(Constants.Deployer.canID);
    canCoder.getConfigurator().apply(canCoderConfigs);

    motorConfigs.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
    motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfigs.Feedback.SensorToMechanismRatio = Constants.Deployer.ratioSM;
    motorConfigs.Feedback.RotorToSensorRatio = Constants.Deployer.ratioRS;
    motorConfigs.CurrentLimits.StatorCurrentLimit = Constants.Deployer.statorCurrentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = Constants.Deployer.supplyCurrentLimit;
    motorConfigs.MotorOutput.Inverted = Constants.Deployer.motorInvert;
    motorConfigs.MotorOutput.NeutralMode = Constants.Deployer.neutralMode;
    pidConfig.kS = Constants.Deployer.kS;
    pidConfig.kV = Constants.Deployer.kV;
    pidConfig.kP = Constants.Deployer.kP;
    pidConfig.kI = Constants.Deployer.kI;
    pidConfig.kD = Constants.Deployer.kD;
    StatusCode deployerConfigStatus = deployerMotor.getConfigurator().apply(motorConfigs);
    StatusCode pidConfigStatus = deployerMotor.getConfigurator().apply(pidConfig);
    canCoder.getConfigurator().apply(canCoderConfigs);
    if (deployerConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + deployerMotor.getDeviceID()
              + " error (Deployer): "
              + deployerConfigStatus.getDescription(),
          false);
    }
    if (pidConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + deployerMotor.getDeviceID()
              + " PID error (Spindexer): "
              + pidConfigStatus.getDescription(),
          false);
    }
    posRotError = subtract(deployerMotor.getPosition().getValueAsDouble(), canCoder.getPosition().getValueAsDouble());
    deployerMotor.setPosition(deployerMotor.getPosition().getValueAsDouble() + posRotError);
  }

  @Override
  public void updateInputs(DeployerIOInputs inputs) {
    // Implementation for updating inputs from the TalonFX hardware

    inputs.connected = deployerMotor.isConnected();

    inputs.angleDeg = toCodeCoords(Units.rotationsToDegrees(deployerMotor.getPosition().getValueAsDouble()));

    inputs.requestedPosDeg = requestedPosDeg;

    inputs.speedRotationsPerSec =
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
  public void enableBrakeMode(Boolean mode) {
    deployerMotor.setNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
  public double subtract(Double pos, Double canPos){
    return pos - canPos;
  }
    private double toCodeCoords(double position) {
    return position + Constants.Deployer.maxGravityDegrees;
  }

  private double toMotorCoords(double position) {
    return position - Constants.Deployer.maxGravityDegrees;
  }
}
