package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.Constants;

public class FlywheelIOTalonFx implements FlywheelIO {

  private TalonFX motor;
  private TalonFX followerMotor;
  private Canandcolor canandcolor;
  private CanandcolorSettings canandcolorConfig = new CanandcolorSettings();
  private double lastRequestedVelocity = -1;

  private TalonFXConfiguration config = new TalonFXConfiguration();
  private VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public FlywheelIOTalonFx() {
    motor = new TalonFX(Constants.Flywheel.motorId);
    followerMotor = new TalonFX(Constants.Flywheel.followerMotorId);

    config.CurrentLimits.StatorCurrentLimit = Constants.Flywheel.statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Flywheel.supplyCurrentLimit;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = Constants.Flywheel.motorToMechanismRatio;

    config.MotorOutput.Inverted = Constants.Flywheel.motorInvert;
    config.MotorOutput.NeutralMode = Constants.Flywheel.neutralMode;

    config.Slot0.kS = Constants.Flywheel.kS;
    config.Slot0.kV = Constants.Flywheel.kV;
    config.Slot0.kP = Constants.Flywheel.kP;
    config.Slot0.kI = Constants.Flywheel.kI;
    config.Slot0.kD = Constants.Flywheel.kD;

    StatusCode leaderConfigStatus = motor.getConfigurator().apply(config);
    StatusCode followerConfigStatus = motor.getConfigurator().apply(config);
    StatusCode followerMotorSetStatus =
        followerMotor.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));

    canandcolorConfig.setColorFramePeriod(10); // Set color frame period to 10ms

    if (Constants.Flywheel.canAndColorEnabled) {
      canandcolor = new Canandcolor(Constants.Flywheel.canandcolorId);
      CanandcolorSettings canandcolorConfigStatus =
          canandcolor.setSettings(canandcolorConfig, 1.0, 5);
      if (!canandcolorConfigStatus.isEmpty()) {
        DriverStation.reportError(
            "Canandcolor "
                + canandcolor.getAddress()
                + " error (Flywheel Sensor): "
                + canandcolorConfigStatus,
            false);
      }
    }

    if (leaderConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + motor.getDeviceID()
              + " config error (Flywheel Leader): "
              + leaderConfigStatus.getDescription(),
          false);
    }
    if (followerConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + followerMotor.getDeviceID()
              + " config error (Flywheel Follower): "
              + followerConfigStatus.getDescription(),
          false);
    }
    if (followerMotorSetStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + followerMotor.getDeviceID()
              + " set follower error (Flywheel Follower): "
              + followerMotorSetStatus.getDescription(),
          false);
    }
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.motorConnected = motor.isConnected();
    inputs.followerMotorConnected = followerMotor.isConnected();

    inputs.requestedMechanismRPS = lastRequestedVelocity;

    inputs.mechanismRPS = motor.getVelocity().getValueAsDouble();
    inputs.followerMechanismRPS = followerMotor.getVelocity().getValueAsDouble();

    inputs.speedMotorRPS = motor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.motorTempCelsius = motor.getDeviceTemp().getValueAsDouble();
    inputs.followerMotorTempCelsius = followerMotor.getDeviceTemp().getValueAsDouble();
    inputs.followerBusCurrentAmps = followerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.leaderStatorAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.followerStatorAmps = followerMotor.getStatorCurrent().getValueAsDouble();
    inputs.followerAppliedVolts = followerMotor.getMotorVoltage().getValueAsDouble();
    inputs.busCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    if (Constants.Flywheel.canAndColorEnabled) {
      inputs.color = new Color(canandcolor.getRed(), canandcolor.getGreen(), canandcolor.getBlue());
      inputs.proximity = canandcolor.getProximity();

      inputs.proximity = canandcolor.getProximity();
      inputs.sensorConnected = canandcolor.isConnected();

      inputs.fuelDetected = inputs.proximity < Constants.Flywheel.minFuelDetectionProximity;
    }
  }

  @Override
  public void setTargetMechanismRPS(double mechanismRPS) {
    if (mechanismRPS != lastRequestedVelocity) {
      if (mechanismRPS == 0) {
        motor.stopMotor();
      } else {
        motor.setControl(velocityRequest.withVelocity(mechanismRPS).withEnableFOC(true));
      }
    }

    lastRequestedVelocity = mechanismRPS;
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

  public TalonFX getTalonFX() {
    return motor;
  }
}
