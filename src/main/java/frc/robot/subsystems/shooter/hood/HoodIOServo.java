package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoHub.Bank;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;
import com.revrobotics.servohub.config.ServoHubConfig;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class HoodIOServo implements HoodIO {
  private ServoHub servoHub;
  private ServoChannel servo;
  private CANcoder encoder;

  private ServoHubConfig config = new ServoHubConfig();

  public HoodIOServo() {
    servoHub = new ServoHub(Constants.Hood.servoChannel);
    servo = servoHub.getServoChannel(ChannelId.fromInt(Constants.Hood.servoChannel));
    encoder = new CANcoder(Constants.Hood.encoderId);

    servoHub.configure(config, ResetMode.kResetSafeParameters);

    ServoChannelConfig channelConfig =
        new ServoChannelConfig(ChannelId.fromInt(Constants.Hood.servoChannel));
    channelConfig.disableBehavior(
        BehaviorWhenDisabled.kDoNotSupplyPower); // Config "coast" mode by disabling channel
    channelConfig.pulseRange(500, 1500, 2500);
    config.apply(ChannelId.fromInt(Constants.Hood.servoChannel), channelConfig);
    servoHub.setBankPulsePeriod(Bank.kBank0_2, 20000); // this value has least latency, why?
    servoHub.setBankPulsePeriod(Bank.kBank3_5, 20000);

    servo.setPowered(true);
    servo.setEnabled(true);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.encoderConnected = encoder.isConnected();
    inputs.encoderRotations = encoder.getPosition().getValueAsDouble();
    inputs.hoodDegrees = inputs.encoderRotations * 360.0 / Constants.Hood.encoderToHoodGearRatio;
    inputs.encoderRPS = encoder.getVelocity().getValueAsDouble();
    inputs.servoAmps = servo.getCurrent();
    inputs.servoEnabled =
        servo.isEnabled(); // Assuming a threshold of 0.1A to determine if the servo is powered
  }

  @Override
  public void setEncoderHomed() {
    encoder.setPosition(0);
  }

  @Override
  public void setPulseWidth(int pulseWidth) {
    servo.setPulseWidth(pulseWidth);
    Logger.recordOutput("Shooter/Hood/pulseWidth", pulseWidth);
  }
}
