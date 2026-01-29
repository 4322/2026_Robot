package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class RollersIOTalonFX implements RollersIO {
  private TalonFX rollersMotor;

  private double previousRequestedVoltage = -999;

  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  public RollersIOTalonFX() {
    rollersMotor = new TalonFX(Constants.Rollers.motorId);

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;

    StatusCode feederConfigStatus = rollersMotor.getConfigurator().apply(motorConfigs);

    if (feederConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + rollersMotor.getDeviceID()
              + " error (Rollers): "
              + feederConfigStatus.getDescription(),
          false);
    }
  }

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    inputs.connected = rollersMotor.isConnected();
    inputs.appliedVoltage = rollersMotor.getMotorVoltage().getValueAsDouble();
    inputs.busCurrentAmps = rollersMotor.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrentAmps = rollersMotor.getStatorCurrent().getValueAsDouble();
    inputs.motorTempCelcius = rollersMotor.getDeviceTemp().getValueAsDouble();
    inputs.speedRotationsPerSec = rollersMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    if (voltage != previousRequestedVoltage) {
      previousRequestedVoltage = voltage;
      rollersMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
    }
  }

  @Override
  public void stopMotor() {
    rollersMotor.stopMotor();
  }

  @Override
  public TalonFX getTalonFX() {
    return rollersMotor;
  }
  @Override
  public void enableBreakMode(boolean mode){
     rollersMotor.setNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
