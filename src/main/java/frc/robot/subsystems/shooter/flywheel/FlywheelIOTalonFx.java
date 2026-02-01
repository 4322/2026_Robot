package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class FlywheelIOTalonFx implements FlywheelIO {

  private TalonFX motor;
  private Canandcolor canandcolor = new Canandcolor(Constants.Flywheel.canandcolorId);
  private double lastRequestedVelocity = -1;

  private TalonFXConfiguration config = new TalonFXConfiguration();
  private VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public FlywheelIOTalonFx() {
    motor = new TalonFX(Constants.Flywheel.FlywheelMotorId);

    config.CurrentLimits.StatorCurrentLimit = Constants.Flywheel.statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Flywheel.supplyCurrentLimit;

    config.MotorOutput.Inverted = Constants.Flywheel.motorInvert;
    config.MotorOutput.NeutralMode = Constants.Flywheel.neutralMode;

    config.Slot0.kS = Constants.Flywheel.kS;
    config.Slot0.kV = Constants.Flywheel.kV;
    config.Slot0.kP = Constants.Flywheel.kP;
    config.Slot0.kI = Constants.Flywheel.kI;
    config.Slot0.kD = Constants.Flywheel.kD;

    StatusCode configStatus = motor.getConfigurator().apply(config);

    if (configStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon " + motor.getDeviceID() + " error (Spindexer): " + configStatus.getDescription(),
          false);
    }
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.motorConnected = motor.isConnected();
    inputs.requestedMechanismRotations = lastRequestedVelocity;
    inputs.actualMechanismRotations =
        motor.getVelocity().getValueAsDouble() / Constants.Flywheel.motorToMechanismRatio;
    inputs.speedMotorRotations = motor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.motorTempCelsius = motor.getDeviceTemp().getValueAsDouble();
    inputs.color = new Color(canandcolor.getRed(), canandcolor.getGreen(), canandcolor.getBlue());
    inputs.distance = canandcolor.getProximity();

    inputs.fuelDetectedOutputting = false;
    inputs.busCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setTargetMechanismRotations(double speedMechanismRotations) {
    if (speedMechanismRotations != lastRequestedVelocity) {
      motor.setControl(
          velocityRequest
              .withVelocity(speedMechanismRotations * Constants.Flywheel.motorToMechanismRatio)
              .withEnableFOC(true));
    }

    lastRequestedVelocity = speedMechanismRotations;
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
}
