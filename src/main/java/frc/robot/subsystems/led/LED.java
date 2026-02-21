package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
  private CANdle leds = new CANdle(Constants.LED.CANdleID);
  private LEDState state = LEDState.DISABLED;

  private boolean climberDeployed = false;
  private boolean autoFuelPickup = false;
  private boolean turretUnwinding = false;

  public enum LEDState {
    DISABLED,
    CLIMBER_DEPLOYED,
    AUTO_FUEL_PICKUP,
    TURRET_UNWINDING,
    SHOOTING_AREA_ACTIVE,
    SHOOTING_AREA_INACTIVE,
    NON_SHOOTING_AREA
  }

  private enum AnimationType {
    COLOR_FLOW,
    FIRE,
    LARSON,
    RAINBOW,
    RGB_FADE,
    SINGLE_FADE,
    SOLID_COLOR,
    STROBE,
    TWINKLE
  }

  public LED() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = Constants.LED.stripType;
    config.LED.BrightnessScalar = Constants.LED.brightnessScalar;
    config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;

    leds.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("LED/State", state.toString());
    if (DriverStation.isDisabled()) {
      setLEDState(LEDState.DISABLED);
    } else if (climberDeployed) {
      setLEDState(LEDState.CLIMBER_DEPLOYED);
    } else if (autoFuelPickup) {
      setLEDState(LEDState.AUTO_FUEL_PICKUP);
    } else if (turretUnwinding) {
      setLEDState(LEDState.TURRET_UNWINDING);
    } else  {
     // TODO add other states
    }
  }

  private void setLEDState(LEDState newState) {
    if (state != newState) {
      state = newState;
      clearLEDs(leds);

      switch (state) {
        case DISABLED -> {
          setLEDs(AnimationType.RAINBOW, 0);
        }

      }
    }
  }

  private void clearLEDs(CANdle leds) {
    for (int i = 0; i < 8; i++) {
      leds.setControl(new EmptyAnimation(i));
    }
  }

  private void setLEDs(AnimationType animation, int slot, int r, int g, int b) {
    switch (animation) {
      case COLOR_FLOW -> {
        leds.setControl(
            new ColorFlowAnimation(Constants.LED.ledStart, Constants.LED.ledEnd)
                .withSlot(slot)
                .withColor(new RGBWColor(r, g, b)));
      }
      case LARSON -> {
        leds.setControl(
            new LarsonAnimation(Constants.LED.ledStart, Constants.LED.ledEnd)
                .withSlot(slot)
                .withColor(new RGBWColor(r, g, b)));
      }
      case SINGLE_FADE -> {
        leds.setControl(
            new SingleFadeAnimation(Constants.LED.ledStart, Constants.LED.ledEnd)
                .withSlot(slot)
                .withColor(new RGBWColor(r, g, b)));
      }
      case SOLID_COLOR -> {
        leds.setControl(
            new SingleFadeAnimation(Constants.LED.ledStart, Constants.LED.ledEnd)
                .withSlot(slot)
                .withColor(new RGBWColor(r, g, b)));
      }
      case STROBE -> {
        leds.setControl(
            new SingleFadeAnimation(Constants.LED.ledStart, Constants.LED.ledEnd)
                .withSlot(slot)
                .withColor(new RGBWColor(r, g, b)));
      }
      case TWINKLE -> {
        leds.setControl(
            new SingleFadeAnimation(Constants.LED.ledStart, Constants.LED.ledEnd)
                .withSlot(slot)
                .withColor(new RGBWColor(r, g, b)));
      }
    }
  }

  private void setLEDs(AnimationType animation, int slot) {
    switch (animation) {
      case RAINBOW -> {
        leds.setControl(
            new RainbowAnimation(Constants.LED.ledStart, Constants.LED.ledEnd).withSlot(slot));
      }
      case COLOR_FLOW -> {
        leds.setControl(
            new RgbFadeAnimation(Constants.LED.ledStart, Constants.LED.ledEnd).withSlot(slot));
      }
      case FIRE -> {
        leds.setControl(
            new FireAnimation(Constants.LED.ledStart, Constants.LED.ledEnd).withSlot(slot));
      }
    }
  }

  private void unsetAllRequests() {
    climberDeployed = false;
    autoFuelPickup = false;
    turretUnwinding = false;
  }

  public void requestClimberDeployed(boolean value) {
    climberDeployed = value;
  }

  public void requestAutoFuelPickup(boolean value) {
    autoFuelPickup = value;
  }

  public void requestTurretUnwinding(boolean value) {
    turretUnwinding = value;
  }

}