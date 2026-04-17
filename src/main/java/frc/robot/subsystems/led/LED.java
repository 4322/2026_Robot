package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
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
    if (DriverStation.isDisabled()) {
      setLEDState(LEDState.DISABLED);
    } else if (shooter.getState() == ShooterState.PRESHOOT) {
      setLEDState(LEDState.PRESHOOT);
    } else if (shooter.getState() == ShooterState.SHOOT) {
      setLEDState(LEDState.SHOOT);
    }
  }

  private void setLEDState(LEDState newState) {
    if (state != newState) {
      state = newState;
      io.clearLEDs();

      switch (state) {
        case DISABLED -> {
          io.setLEDs(AnimationType.RAINBOW, 0);
        }
        case INACTIVE -> {
          
        }
        case NON_SHOOTING_AREA -> {
          io.setLEDs(AnimationType.COLOR_FLOW, 0, 255, 0, 0);
        }
      }
    }
  }
}
