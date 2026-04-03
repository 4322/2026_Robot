package frc.robot.subsystems.shooter.spindexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class SpindexerIOTalonFx implements SpindexerIO {
  private TalonFX motor;
  private double lastRequestedVelocity = -1;

  private TalonFXConfiguration config = new TalonFXConfiguration();
  private VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temp;

  private final Debouncer motorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public SpindexerIOTalonFx() {
    motor = new TalonFX(Constants.Spindexer.spindexerMotorId, Constants.CANivore.CANBus);

    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = Constants.Spindexer.statorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Spindexer.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = Constants.Spindexer.supplyCurrentLowerLimit;
    config.CurrentLimits.SupplyCurrentLowerTime = Constants.Spindexer.supplyCurrentLowerTime;

    config.MotorOutput.Inverted = Constants.Spindexer.motorInvert;
    config.MotorOutput.NeutralMode = Constants.Spindexer.neutralMode;

    config.Slot0.kS = Constants.Spindexer.kS;
    config.Slot0.kV = Constants.Spindexer.kV;
    config.Slot0.kP = Constants.Spindexer.kP;
    config.Slot0.kI = Constants.Spindexer.kI;
    config.Slot0.kD = Constants.Spindexer.kD;

    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;

    config.Feedback.SensorToMechanismRatio = Constants.Spindexer.motorToMechanismRatio;

    StatusCode configStatus = motor.getConfigurator().apply(config);

    if (configStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon " + motor.getDeviceID() + " error (Spindexer): " + configStatus.getDescription(),
          false);
    }

    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    temp = motor.getDeviceTemp();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, appliedVolts, statorCurrent, supplyCurrent, temp);
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    var motorStatus =
        BaseStatusSignal.refreshAll(velocity, appliedVolts, statorCurrent, supplyCurrent, temp);

    inputs.motorConnected = motorConnectedDebounce.calculate(motorStatus.isOK());
    inputs.voltage = appliedVolts.getValueAsDouble();
    inputs.mechanismRPS = velocity.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.motorTempC = temp.getValueAsDouble();
  }

  @Override
  public void setTargetMechanismRotations(double velocity) {
    if (velocity != lastRequestedVelocity) {
      if (velocity == 0) {
        motor.stopMotor();
      } else {
        motor.setControl(velocityRequest.withVelocity(velocity).withEnableFOC(true));
      }
    }

    lastRequestedVelocity = velocity;
  }

  @Override
  public void stop() {
    lastRequestedVelocity = 0;
    motor.stopMotor();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    motor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
