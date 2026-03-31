package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

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
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;

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
    CANconfigOne.MagnetSensor.MagnetOffset = Constants.Turret.CANCoderOneOffsetRot;
    CANconfigTwo.MagnetSensor.MagnetOffset = Constants.Turret.CANCoderTwoOffsetRot;
    CANconfigOne.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1; // range 0 to 1.0
    CANconfigTwo.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    CANconfigOne.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    CANconfigTwo.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

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
    inputs.encoderOneRot = CANcoderOne.getAbsolutePosition().getValueAsDouble();
    inputs.encoderTwoRot = CANcoderTwo.getAbsolutePosition().getValueAsDouble();
    inputs.motorConnected = turretMotor.isConnected();
    inputs.motorRPS = turretMotor.getVelocity().getValueAsDouble();
    inputs.supplyCurrentAmps = turretMotor.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrentAmps = turretMotor.getStatorCurrent().getValueAsDouble();
    inputs.tempCelsius = turretMotor.getDeviceTemp().getValueAsDouble();
    inputs.appliedVoltage = turretMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setBrakeMode(boolean mode) {
    turretMotor.setNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setAngle(double degs, double ffRadPerSec) {
    turretMotor.setControl(
        new MotionMagicVoltage(Units.degreesToRotations(degs))
            .withEnableFOC(true)
            .withSlot(0)
            .withFeedForward(ffRadPerSec * Constants.Turret.rotCompensationkV));
  }

  @Override
  public void setPosition(double rot) {
    turretMotor.setPosition(rot);
  }
}
