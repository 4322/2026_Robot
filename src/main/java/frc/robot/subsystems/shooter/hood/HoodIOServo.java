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
import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;

public class HoodIOServo implements HoodIO {
  private ServoHub servoHub;
  private ServoChannel servo;
  private CANcoder encoder;
  private int currentRequested = 1500;

  private ServoHubConfig config = new ServoHubConfig();

  public HoodIOServo() {
    servoHub = new ServoHub(Constants.Hood.servoChannel);
    servo = servoHub.getServoChannel(ChannelId.fromInt(Constants.Hood.servoChannel));
    encoder = new CANcoder(Constants.Hood.encoderId);

    configServo();
  }

  private void configServo() {
    servoHub.configure(config, ResetMode.kResetSafeParameters);

    ServoChannelConfig channelConfig =
        new ServoChannelConfig(ChannelId.fromInt(Constants.Hood.servoChannel));
    channelConfig.disableBehavior(
        BehaviorWhenDisabled.kDoNotSupplyPower); // Config "coast" mode by disabling channel
    channelConfig.pulseRange(1000, 1500, 2000); // Default PWM pulses recommended by REV
    config.apply(ChannelId.fromInt(Constants.Hood.servoChannel), channelConfig);

    servoHub.setBankPulsePeriod(Bank.kBank0_2, 20000); // TODO set this

    servo.setPowered(true);

    // Enables "brake" mode on servos
    servo.setEnabled(true);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.encoderConnected = encoder.isConnected();
    inputs.currentPulseWidth = currentRequested;
    inputs.encoderRotations =
        encoder.getPosition().getValueAsDouble(); // Convert degrees to rotations
    inputs.degrees =
        inputs.encoderRotations * 360.0 / Constants.Hood.gearRatio; // Convert rotations to degrees
    inputs.encoderRPS = encoder.getVelocity().getValueAsDouble();
    inputs.servoEnabled =
        servo.isEnabled(); // Assuming a threshold of 0.1A to determine if the servo is powered
  }

  @Override
  public void setEncoderHomed() {
    encoder.setPosition(0);
  }

  @Override
  public void setServoVelocity(double velocity) {
    double range =  (500 - Constants.Hood.kSPulsewidth);
    double pulseVelocity = -(MathUtil.clamp(velocity, -1, 1) * range);
    this.currentRequested = (int) (1500 + (Constants.Hood.kSPulsewidth * Math.signum(pulseVelocity)) + pulseVelocity);
    servo.setPulseWidth(currentRequested);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    servo.setEnabled(brake);
  }
}
