package frc.robot.subsystems.intake.deployer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class DeployerIOTalonFX implements DeployerIO {
  private TalonFX deployerMotor;
  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
  public double requestedPosDeg;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temp;

  private final Debouncer motorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public DeployerIOTalonFX() {
    deployerMotor = new TalonFX(Constants.Deployer.motorId, Constants.CANivore.CANBus);


    motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motorConfigs.Feedback.SensorToMechanismRatio =
        Constants.Deployer.sensorToMechanismRatio * Constants.Deployer.RotorToSensorRatio;
    motorConfigs.Feedback.RotorToSensorRatio = Constants.Deployer.RotorToSensorRatio;
    motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;
    motorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    motorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    motorConfigs.CurrentLimits.StatorCurrentLimit = Constants.Deployer.statorCurrentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = Constants.Deployer.supplyCurrentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLowerTime = 0;

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

    StatusCode deployerConfigStatus = deployerMotor.getConfigurator().apply(motorConfigs);
    if (deployerConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + deployerMotor.getDeviceID()
              + " error (Deployer): "
              + deployerConfigStatus.getDescription(),
          false);
    }

   
    deployerMotor.setPosition(
        deployerMotor.getPosition().getValueAsDouble() / Constants.Deployer.sensorToMechanismRatio);

    position = deployerMotor.getPosition();
    velocity = deployerMotor.getVelocity();
    appliedVolts = deployerMotor.getMotorVoltage();
    statorCurrent = deployerMotor.getStatorCurrent();
    supplyCurrent = deployerMotor.getSupplyCurrent();
    temp = deployerMotor.getDeviceTemp();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, statorCurrent, supplyCurrent, temp);
    ParentDevice.optimizeBusUtilizationForAll(deployerMotor);
  }

  @Override
  public void updateInputs(DeployerIOInputs inputs) {
    // Implementation for updating inputs from the TalonFX hardware
    var motorStatus =
        BaseStatusSignal.refreshAll(
            position, velocity, appliedVolts, statorCurrent, supplyCurrent, temp);

    inputs.motorConnected = motorConnectedDebounce.calculate(motorStatus.isOK());
    inputs.angleDeg = Units.rotationsToDegrees(position.getValueAsDouble());
    inputs.motorDegreesPerSec = Units.rotationsToDegrees(velocity.getValueAsDouble());
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.motorTempCelcius = temp.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();

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
  public void setVoltage(double voltage) {
    deployerMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
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
  public void seedPosition(double newAngleDeg) {
    deployerMotor.setPosition(Units.degreesToRotations(newAngleDeg));
  }

  @Override
  public void setBrakeMode(boolean mode) {
    deployerMotor.setNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
