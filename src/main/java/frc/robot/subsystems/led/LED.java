package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.util.HubShiftUtil;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
  private LEDState state = LEDState.DISABLED;

  private boolean flash = false;

  private LEDIO io;
  private Shooter shooter;

  public enum LEDState {
    DISABLED,
    INACTIVE,
    NON_SHOOTING_AREA,
    PRESHOOT,
    SHOOT,
    IDLE
  }

  public enum AnimationType {
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

  public LED(LEDIO io, Shooter shooter) {
    this.io = io;
    this.shooter = shooter;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("LED/State", state.toString());
    flash = HubShiftUtil.fiveSecondsLeft();

    if (DriverStation.isDisabled()) {
      setLEDState(LEDState.DISABLED);
    } else if (shooter.getState() == ShooterState.PRESHOOT) {
      setLEDState(LEDState.PRESHOOT);
    } else if (shooter.getState() == ShooterState.SHOOT) {
      setLEDState(LEDState.SHOOT);
    } else if (!HubShiftUtil.getOfficialShiftInfo().active()) {
      setLEDState(LEDState.INACTIVE);
    } else if (shooter.isInShootingArea()) {
      setLEDState(LEDState.NON_SHOOTING_AREA);
    } else {
      setLEDState(LEDState.IDLE);
    }
  }

  private void setLEDState(LEDState newState) {
    if (state != newState) {
      state = newState;
      io.clearLEDs();
      if (flash) {
        io.setLEDs(AnimationType.STROBE, 1);
      }

      switch (state) {
        case DISABLED -> {
          io.setLEDs(AnimationType.RAINBOW, 0);
        }
        case INACTIVE -> {
          io.setLEDs(AnimationType.SOLID_COLOR, 0, 255, 0, 0);
        }
        case NON_SHOOTING_AREA -> {
          io.setLEDs(AnimationType.LARSON, 0, 255, 170, 0);
        }
        case PRESHOOT -> {
          io.setLEDs(AnimationType.LARSON, 0, 100, 255, 0);
        }
        case SHOOT -> {
          io.setLEDs(AnimationType.RAINBOW, 0);
        }
        case IDLE -> {
          io.setLEDs(AnimationType.SOLID_COLOR, 0, 0, 0, 255);
        }
      }
    }
  }
}
