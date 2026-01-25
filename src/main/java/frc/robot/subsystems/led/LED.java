package frc.robot.subsystems.led;

import com.ctre.phoenix6.hardware.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class LED extends SubsystemBase {
    private CANdle led = new CANdle(Constants.LED.CANdleID);
    private LEDState state = LEDState.UNKNOWN;

    public enum LEDState {
        UNKNOWN,
        IDLE

    }

    public LED() {}
    
}
