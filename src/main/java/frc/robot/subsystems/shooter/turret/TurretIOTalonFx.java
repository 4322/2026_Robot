package frc.robot.subsystems.shooter.turret;

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
import frc.robot.util.PhoenixUtil;

public class TurretIOTalonFx implements TurretIO {
  private TalonFX turretMotor;
  private CANcoder CANcoderOne;
  private CANcoder CANcoderTwo;
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private CANcoderConfiguration CANconfigOne = new CANcoderConfiguration();
  private CANcoderConfiguration CANconfigTwo = new CANcoderConfiguration();

  public TurretIOTalonFx() {
    turretMotor = new TalonFX(Constants.Turret.motorId, Constants.CANivore.CANBus);
    CANcoderOne = new CANcoder(Constants.Turret.CANCoderOneId, Constants.CANivore.CANBus);
    CANcoderTwo = new CANcoder(Constants.Turret.CANCoderTwoId, Constants.CANivore.CANBus);
    config.CurrentLimits.StatorCurrentLimit = Constants.Turret.statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Turret.supplyCurrentLimit;
    config.MotorOutput.Inverted = Constants.Turret.motorInvert;
    config.MotorOutput.NeutralMode = Constants.Turret.neutralMode;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = Constants.Turret.turretGearRatio;

    config.Slot0.kS = Constants.Turret.kS;
    config.Slot0.kV = Constants.Turret.kV;
    config.Slot0.kP = Constants.Turret.kP;
    config.Slot0.kI = Constants.Turret.kI;
    config.Slot0.kD = Constants.Turret.kD;

    config.HardwareLimitSwitch.ReverseLimitEnable = false;
    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(Constants.Turret.minPhysicalLimitDeg);
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(Constants.Turret.maxPhysicalLimitDeg);

    double CANCoderOneOffsetRot =
        Constants.Turret.CANCoderOneOffsetCount / (double) Constants.Turret.CANCoderResolution
            - 0.25;
    if (CANCoderOneOffsetRot < 0) {
      CANCoderOneOffsetRot++;
    }
    double CANCoderTwoOffsetRot =
        Constants.Turret.CANCoderTwoOffsetCount / (double) Constants.Turret.CANCoderResolution
            - 0.25;
    if (CANCoderTwoOffsetRot < 0) {
      CANCoderTwoOffsetRot++;
    }
    CANconfigOne.MagnetSensor.MagnetOffset = CANCoderOneOffsetRot;
    CANconfigTwo.MagnetSensor.MagnetOffset = CANCoderTwoOffsetRot;
    CANconfigOne.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1; // range 0 to 1.0
    CANconfigTwo.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

    config.MotionMagic.MotionMagicCruiseVelocity = Constants.Turret.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = Constants.Turret.motionMagicAcceleration;

    StatusCode configStatus = turretMotor.getConfigurator().apply(config);
    StatusCode CANcoderOneStatus = CANcoderOne.getConfigurator().apply(CANconfigOne);
    StatusCode CANcoderTwoStatus = CANcoderTwo.getConfigurator().apply(CANconfigTwo);

    if (configStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + turretMotor.getDeviceID()
              + " error (Turret): "
              + configStatus.getDescription(),
          false);
    }
    if (CANcoderOneStatus != StatusCode.OK) {
      DriverStation.reportError(
          "CANCoderOne "
              + CANcoderOne.getDeviceID()
              + " error (Turret): "
              + CANcoderOneStatus.getDescription(),
          false);
    }
    if (CANcoderTwoStatus != StatusCode.OK) {
      DriverStation.reportError(
          "CANCoderTwo "
              + CANcoderTwo.getDeviceID()
              + " error (Turret): "
              + CANcoderTwoStatus.getDescription(),
          false);
    }

    try {
      // wait for encoder positions to be received
      Thread.sleep(100);
    } catch (InterruptedException e) {
    }
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretDegs = Units.rotationsToDegrees(turretMotor.getPosition().getValueAsDouble());
    inputs.encoderOneCount =
        (int)
            (CANcoderOne.getAbsolutePosition().getValueAsDouble()
                * Constants.Turret.CANCoderResolution);
    inputs.encoderTwoCount =
        (int)
            (CANcoderTwo.getAbsolutePosition().getValueAsDouble()
                * Constants.Turret.CANCoderResolution);
    inputs.motorConnected = turretMotor.isConnected();
    inputs.motorRPS = turretMotor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = turretMotor.getSupplyVoltage().getValueAsDouble();
    inputs.TempCelsius = turretMotor.getDeviceTemp().getValueAsDouble();
    inputs.statorVolts = turretMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setBrakeMode(boolean mode) {
    turretMotor.setNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setAngle(double degs) {
    turretMotor.setControl(
        new MotionMagicVoltage(Units.degreesToRotations(degs)).withEnableFOC(true).withSlot(0));
  }

  @Override
  public void setPosition(double rot) {
    PhoenixUtil.tryUntilOk(5, () -> turretMotor.setPosition(rot));
  }
}
