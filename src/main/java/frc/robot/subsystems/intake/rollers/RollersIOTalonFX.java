package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class RollersIOTalonFX implements RollersIO {
  private TalonFX leaderMotor;
  private TalonFX followerMotor;

  private Double previousRequestedVoltage;

  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  public RollersIOTalonFX() {
    leaderMotor = new TalonFX(Constants.Rollers.leaderMotorId);
    followerMotor = new TalonFX(Constants.Rollers.followerMotorId);

    motorConfigs.MotorOutput.NeutralMode = Constants.Rollers.neutralMode;

    motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;
    motorConfigs.CurrentLimits.StatorCurrentLimit = Constants.Rollers.statorCurrentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = Constants.Rollers.supplyCurrentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    
    motorConfigs.MotorOutput.Inverted = Constants.Rollers.leaderMotorInvert;
    motorConfigs.MotorOutput.NeutralMode = Constants.Rollers.neutralMode;
    StatusCode leaderConfigStatus = leaderMotor.getConfigurator().apply(motorConfigs);
    StatusCode followerConfigStatus = followerMotor.getConfigurator().apply(motorConfigs);
    StatusCode followerMotorSetStatus =
        followerMotor.setControl(
            new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    if (leaderConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + leaderMotor.getDeviceID()
              + " config error (Rollers Leader): "
              + leaderConfigStatus.getDescription(),
          false);
    }
    if (followerConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + followerMotor.getDeviceID()
              + " config error (Rollers Follower): "
              + followerConfigStatus.getDescription(),
          false);
    }
    if (followerMotorSetStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + followerMotor.getDeviceID()
              + " set follower error (Rollers Follower): "
              + followerMotorSetStatus.getDescription(),
          false);
    }
  }

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    inputs.leaderConnected = leaderMotor.isConnected();
    inputs.leaderVolts = leaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leaderMotor.getSupplyCurrent().getValueAsDouble();
    inputs.leaderStatorCurrentAmps = leaderMotor.getStatorCurrent().getValueAsDouble();
    inputs.leaderMotorTempCelcius = leaderMotor.getDeviceTemp().getValueAsDouble();
    inputs.leaderControllerTempCelcius = leaderMotor.getProcessorTemp().getValueAsDouble();
    inputs.leaderRotationsPerSec = leaderMotor.getVelocity().getValueAsDouble();

    inputs.followerConnected = followerMotor.isConnected();
    inputs.followerVolts = followerMotor.getMotorVoltage().getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.followerStatorCurrentAmps = followerMotor.getStatorCurrent().getValueAsDouble();
    inputs.followerMotorTempCelcius = followerMotor.getDeviceTemp().getValueAsDouble();
    inputs.followerControllerTempCelcius = followerMotor.getProcessorTemp().getValueAsDouble();
    inputs.followerRotationsPerSec = followerMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    if ((previousRequestedVoltage == null) || (previousRequestedVoltage != voltage)) {
      previousRequestedVoltage = voltage;
      leaderMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
    }
  }

  @Override
  public void stopMotor() {
    leaderMotor.stopMotor();
    previousRequestedVoltage = null;
  }

  @Override
  public TalonFX getTalonFX() {
    return leaderMotor;
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    leaderMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
