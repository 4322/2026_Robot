package frc.robot.subsystems.shooter.hood;

import com.revrobotics.REVLibError;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoHub.Bank;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;
import com.revrobotics.servohub.config.ServoHubConfig;
import frc.robot.constants.Constants;

public class HoodIOServo {
  private ServoHub servoHub;
  private ServoChannel servo;

  private ServoHubConfig config = new ServoHubConfig();

  public HoodIOServo() {
    servoHub = new ServoHub(Constants.Hood.servoHubId);
    servo = servoHub.getServoChannel(ChannelId.fromInt(Constants.Hood.servoChannelId));

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

    servoHub.setBankPulsePeriod(Bank.kBank0_2, 20000); // TODO set this

    servo.setPowered(true);

    // Enables "brake" mode on servos
    servo.setEnabled(true);

    // Set default position
    servo.setPulseWidth(Constants.Hood.servoDefaultPWM);

    return servoHub.configure(config, ResetMode.kResetSafeParameters);
  }

  public void setServoPosition(boolean pull) {
    /* TODO figure out what to do for this
    servo.setPulseWidth(
        pull ? Constants.Hood.servoDefaultPWM : Constants.Hood.servoDefaultPWM); */
  }
}
