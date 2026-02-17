package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
  private double CANCoderOneMod;
  private double CANCoderTwoMod;
  private double turretMod;

  public TurretIOTalonFx() {
    turretMotor = new TalonFX(Constants.Turret.motorId, Constants.CANbus.CANBus);
    CANcoderOne = new CANcoder(Constants.Turret.CANCoderOneId, Constants.CANbus.CANBus);
    CANcoderTwo = new CANcoder(Constants.Turret.CANCoderTwoId, Constants.CANbus.CANBus);
    config.CurrentLimits.StatorCurrentLimit = Constants.Turret.statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Turret.supplyCurrentLimit;
    config.MotorOutput.Inverted = Constants.Turret.motorInvert;
    config.MotorOutput.NeutralMode = Constants.Turret.neutralMode;

    config.Slot0.kS = Constants.Turret.kS;
    config.Slot0.kV = Constants.Turret.kV;
    config.Slot0.kP = Constants.Turret.kP;
    config.Slot0.kI = Constants.Turret.kI;
    config.Slot0.kD = Constants.Turret.kD;

    config.HardwareLimitSwitch.ForwardLimitEnable = true;
    config.HardwareLimitSwitch.ReverseLimitEnable = true;

    StatusCode configStatus = turretMotor.getConfigurator().apply(config);
    StatusCode CANcoderStatus = CANcoderOne.getConfigurator().apply(CANconfigOne);
    StatusCode CANcoderStatusTwo = CANcoderTwo.getConfigurator().apply(CANconfigTwo);

    if (configStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + turretMotor.getDeviceID()
              + " error (Turret): "
              + configStatus.getDescription(),
          false);
    }
    if (CANcoderStatus != StatusCode.OK) {
      DriverStation.reportError(
          "CANCoderOne "
              + CANcoderOne.getDeviceID()
              + " error (CANCoderOne): "
              + CANcoderStatus.getDescription(),
          false);
    }
    if (CANcoderStatusTwo != StatusCode.OK) {
      DriverStation.reportError(
          "CANCoderTwo "
              + CANcoderTwo.getDeviceID()
              + " error (CANCoderTwo): "
              + CANcoderStatusTwo.getDescription(),
          false);
    }
    turretMotor.setPosition(getAngle());
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretDegs = Units.rotationsToDegrees(turretMotor.getPosition().getValueAsDouble());
    inputs.encoderOneRotations = CANcoderOne.getPosition().getValueAsDouble();
    inputs.encoderTwoRotations = CANcoderTwo.getPosition().getValueAsDouble();
    inputs.motorConnected = turretMotor.isConnected();
    inputs.speedMotorRotations = turretMotor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = turretMotor.getSupplyVoltage().getValueAsDouble();
    inputs.motorTempCelsius = turretMotor.getDeviceTemp().getValueAsDouble();
    inputs.statorVolts = turretMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setBrakeMode(boolean mode) {
    turretMotor.setNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  public double getAngle() {
    CANCoderOneMod =
        CANcoderOne.getPosition().getValueAsDouble() % Constants.Turret.CANCoderOneRatio;
    CANCoderTwoMod =
        CANcoderTwo.getPosition().getValueAsDouble() % Constants.Turret.CANCoderTwoRatio;

    turretMod = (((10 * CANCoderOneMod) + (36 * CANCoderTwoMod)) % 45);

    return turretMod;
  }

  public void setAngle(double degs) {
    double targetPosition =
        Units.degreesToRotations(degs) * Constants.Turret.turretGearRatio
            - Units.degreesToRotations(Constants.Turret.midPointPhysicalDeg);
    turretMotor.setControl(new MotionMagicVoltage(Units.degreesToRotations(degs)));
  }
}
