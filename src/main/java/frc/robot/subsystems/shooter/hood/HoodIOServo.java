package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.REVLibError;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoHub.Bank;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;
import com.revrobotics.servohub.config.ServoHubConfig;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOInputs;

public class HoodIOServo implements HoodIO {
    private ServoHub servoHub;
    private ServoChannel servo;
    private CANcoder encoder;

    private ServoHubConfig config = new ServoHubConfig();

    public HoodIOServo() {
        servoHub = new ServoHub(Constants.Hood.motorId);
        servo = servoHub.getServoChannel(ChannelId.fromInt(Constants.Hood.motorId));
        encoder = new CANcoder(Constants.Hood.encoderId);
        

        REVLibError configStatus = configServo();

    }

    private REVLibError configServo() {
    for (int i = 0; i < 6; i++) {
      ServoChannelConfig channelConfig = new ServoChannelConfig(ChannelId.fromInt(i));
      channelConfig.disableBehavior(
          BehaviorWhenDisabled.kDoNotSupplyPower); // Config "coast" mode by disabling channel
      channelConfig.pulseRange(500, 1500, 2500); // Default PWM pulses recommended by REV
      config.apply(ChannelId.fromInt(i), channelConfig);
    }

    servoHub.setBankPulsePeriod(Bank.kBank0_2, 20000); //TODO set this

    servo.setPowered(true);

    // Enables "brake" mode on servos
    servo.setEnabled(true);


    return servoHub.configure(config, ResetMode.kResetSafeParameters);
  }
  
  @Override
  public void setServoPosition(int angle) { 
    servo.setPulseWidth(angle);
      }
  @Override
  public void setEncoderPosition(int angle) {
    encoder.setPosition(angle);
  }
    
@Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.encoderConnected = encoder.isConnected();
    inputs.currentPulseWidth = servo.getPulseWidth();
    inputs.angleDeg = encoder.getAbsolutePosition().getValueAsDouble();
  }
    
    
}
