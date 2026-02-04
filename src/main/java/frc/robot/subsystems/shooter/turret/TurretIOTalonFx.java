package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class TurretIOTalonFx implements TurretIO {
    private TalonFX turretMotor;
    private CANcoder CANcoderOne;
    private CANcoder CANcoderTwo;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private CANcoderConfiguration CANconfigOne = new CANcoderConfiguration();
    private CANcoderConfiguration CANconfigTwo = new CANcoderConfiguration();
    public TurretIOTalonFx(){
    turretMotor = new TalonFX(Constants.Turret.motorId);
    config.CurrentLimits.StatorCurrentLimit = Constants.Turret.statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Turret.supplyCurrentLimit;
    config.MotorOutput.Inverted = Constants.Turret.motorInvert;
    config.MotorOutput.NeutralMode = Constants.Turret.neutralMode;

    config.Slot0.kS = Constants.Turret.kS;
    config.Slot0.kV = Constants.Turret.kV;
    config.Slot0.kP = Constants.Turret.kP;
    config.Slot0.kI = Constants.Turret.kI;
    config.Slot0.kD = Constants.Turret.kD;

    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;

    StatusCode configStatus = turretMotor.getConfigurator().apply(config);

    if (configStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon " + turretMotor.getDeviceID() + " error (Turret): " + configStatus.getDescription(),
          false);
    }
    } 
}
