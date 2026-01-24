package frc.robot.subsystems.Intake.Deployer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class DeployerIOTalonFX implements DeployerIO {
  private TalonFX deployerMotor;
  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
  /*
  public DeployerIOTalonFX() {
    deployerMotor = new TalonFX(Constants.Deployer.frontMotorID);

    // Setup config objects
    motorConfigs.CurrentLimits.StatorCurrentLimit = Constants.Deployer.statorCurrentLimitAmps;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = Constants.Deployer.supplyCurrentLimitAmps;

    motorConfigs.MotorOutput.Inverted = Constants.Deployer.deployerInversion;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfigs.Slot0.kD = Constants.Deployer.deploy_kD;
    motorConfigs.Slot0.kP = Constants.Deployer.deploy_kP;
    motorConfigs.Slot0.kI = Constants.Deployer.deploy_kI;
    motorConfigs.Slot0.kG = Constants.Deployer.kG;
    motorConfigs.Slot0.GravityType = GravityTypeValue.Deployer_Static;

    motorConfigs.Slot1.kD = Constants.Deployer.retract_kD;
    motorConfigs.Slot1.kP = Constants.Deployer.retract_kP;
    motorConfigs.Slot1.kI = Constants.Deployer.retract_kI;
    motorConfigs.Slot1.kG = Constants.Deployer.kG;
    motorConfigs.Slot1.GravityType = GravityTypeValue.Deployer_Static;

    motorConfigs.MotionMagic.MotionMagicAcceleration =
        metersToRotations(Constants.Deployer.deployAccelerationMetersPerSec2);
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity =
        metersToRotations(Constants.Deployer.deployVelocityMetersPerSec);
    motorConfigs.MotionMagic.MotionMagicJerk = Constants.Deployer.motionMagicJerk;

    motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;

    StatusCode deployerConfigStatus = deployerMotor.getConfigurator().apply(motorConfigs);
    motorConfigs.MotorOutput.Inverted = Constants.Deployer.followerInversion;

    if (deployerConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + deployerMotor.getDeviceID()
              + " error (Right Deployer): "
              + deployerConfigStatus.getDescription(),
          false);
    }
  }
  */

  @Override
  public void updateInputs(DeployerIOInputs inputs) {
    // Implementation for updating inputs from the TalonFX hardware
    /*
    inputs.deployerConnected = deployerMotor.isConnected();

    inputs.requestedPosRotations = lastRequestedPosRotations;

    inputs.deployerHeightMeters = rotationsToMeters(deployerMotor.getPosition().getValueAsDouble());

    inputs.deployerVelocityMetersPerSecond =
        rotationsToMeters(deployerMotor.getVelocity().getValueAsDouble());

    inputs.deployerSupplyAmps = deployerMotor.getSupplyCurrent().getValueAsDouble();

    inputs.deployerStatorAmps = deployerMotor.getStatorCurrent().getValueAsDouble();

    inputs.deployerTempCelcius = deployerMotor.getDeviceTemp().getValueAsDouble();

    inputs.deployerVoltage = deployerMotor.getMotorVoltage().getValueAsDouble();
    inputs.deployerEncoderRotations = deployerMotor.getPosition().getValueAsDouble();
    */
  }

  @Override
  public void setPosition(double DeployerPositionMeters) {}
  /*
  lastRequestedPosRotations = metersToRotations(DeployerPositionMeters);
  deployerMotor.setPosition(lastRequestedPosRotations);
  followerMotor.setPosition(lastRequestedPosRotations);
  stop();*/

  /*
  @Override
  public void requestretractHeightMeters(double heightMeters) {
    lastRequestedPosMeters = heightMeters;
    lastRequestedPosRotations = metersToRotations(heightMeters);
    deployerMotor.setControl(
        new MotionMagicVoltage(lastRequestedPosRotations).withSlot(1).withEnableFOC(true));
  }

  @Override
  public void setVoltage(double voltage) {
    deployerMotor.setVoltage(voltage);
    lastRequestedPosRotations = -1;
  }

  @Override
  public void stop() {
    deployerMotor.stopMotor();
    lastRequestedPosRotations = -1;
  }

  @Override
  public TalonFX getTalonFX() {
    return deployerMotor;
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    deployerMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  public void degreesToRotations(){}
  */
}
