package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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

public class HoodIOTalonFx implements HoodIO {
  private TalonFX hoodMotor;
  private TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temp;

  private final Debouncer motorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public HoodIOTalonFx() {
    hoodMotor = new TalonFX(Constants.Hood.motorId, Constants.CANivore.CANBus);

    config.CurrentLimits.StatorCurrentLimit = Constants.Hood.statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Hood.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;

    config.MotorOutput.Inverted = Constants.Hood.motorInvert;
    config.MotorOutput.NeutralMode = Constants.Hood.neutralMode;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio =
        Constants.Hood.motorToEncoderGearRatio * Constants.Hood.encoderToHoodGearRatio;
    ;

    config.Slot0.kS = Constants.Hood.kS;
    config.Slot0.kV = Constants.Hood.kV;
    config.Slot0.kP = Constants.Hood.kP;
    config.Slot0.kI = Constants.Hood.kI;
    config.Slot0.kD = Constants.Hood.kD;

    config.MotionMagic.MotionMagicCruiseVelocity = Constants.Hood.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = Constants.Hood.motionMagicAcceleration;
    StatusCode configStatus = hoodMotor.getConfigurator().apply(config);
    if (configStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon " + hoodMotor.getDeviceID() + " error (Hood): " + configStatus.getDescription(),
          false);
    }

    position = hoodMotor.getPosition();
    velocity = hoodMotor.getVelocity();
    appliedVolts = hoodMotor.getMotorVoltage();
    statorCurrent = hoodMotor.getStatorCurrent();
    supplyCurrent = hoodMotor.getSupplyCurrent();
    temp = hoodMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, statorCurrent, supplyCurrent, temp);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    var motorStatus =
        BaseStatusSignal.refreshAll(
            position, velocity, appliedVolts, statorCurrent, supplyCurrent, temp);

    inputs.motorConnected = motorConnectedDebounce.calculate(motorStatus.isOK());
    inputs.hoodDegrees = Units.rotationsToDegrees(position.getValueAsDouble());
    inputs.hoodRPS = velocity.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
  }

  @Override
  public void setHomed() {
    hoodMotor.setPosition(0);
  }

  @Override
  public void setVoltage(double voltage) {
    hoodMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  @Override
  public void setBrakeMode(boolean mode) {
    hoodMotor.setNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setAngle(double degs) {
    hoodMotor.setControl(
        new MotionMagicVoltage(Units.degreesToRotations(degs)).withEnableFOC(true).withSlot(0));
  }
}
