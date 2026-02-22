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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.util.HubTracker;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
  private CANdle leds = new CANdle(Constants.LED.CANdleID);
  private LEDState state = LEDState.DISABLED;

  private boolean climberDeployed = false;
  private boolean autoFuelPickup = false;
  private boolean turretUnwinding = false;

  private Drive drive;

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
    SINGLE_FADE,
    SOLID_COLOR,
    STROBE,
    TWINKLE,
    RGB_FADE
  }

  public LED(Drive drive) {
    this.drive = drive;

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
    } else if (AreaManager.isShootingArea(drive.getPose().getTranslation())) {
      if (HubTracker.isAbleToShoot()) {
        setLEDState(LEDState.SHOOTING_AREA_ACTIVE);
      } else {
        setLEDState(LEDState.SHOOTING_AREA_INACTIVE);
      }

    } else {
      setLEDState(LEDState.NON_SHOOTING_AREA);
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
        case CLIMBER_DEPLOYED -> {}
        case AUTO_FUEL_PICKUP -> {
          setLEDs(AnimationType.STROBE, 0, 0, 0, 255);
        }
        case TURRET_UNWINDING -> {
          setLEDs(AnimationType.LARSON, 0, 255, 0, 255);
        }
        case SHOOTING_AREA_ACTIVE -> {
          setLEDs(AnimationType.COLOR_FLOW, 0, 0, 255, 0);
        }
        case SHOOTING_AREA_INACTIVE -> {
          setLEDs(AnimationType.COLOR_FLOW, 0, 255, 255, 0);
        }
        case NON_SHOOTING_AREA -> {
          setLEDs(AnimationType.COLOR_FLOW, 0, 255, 0, 0);
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
      case RAINBOW, FIRE, RGB_FADE -> {}
    }
  }

  private void setLEDs(AnimationType animation, int slot) {
    switch (animation) {
      case RAINBOW -> {
        leds.setControl(
            new RainbowAnimation(Constants.LED.ledStart, Constants.LED.ledEnd).withSlot(slot));
      }
      case RGB_FADE -> {
        leds.setControl(
            new RgbFadeAnimation(Constants.LED.ledStart, Constants.LED.ledEnd).withSlot(slot));
      }
      case FIRE -> {
        leds.setControl(
            new FireAnimation(Constants.LED.ledStart, Constants.LED.ledEnd).withSlot(slot));
      }
      case LARSON, SINGLE_FADE, SOLID_COLOR, STROBE, TWINKLE, COLOR_FLOW -> {}
    }
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
