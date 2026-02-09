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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.Constants;

import frc.robot.subsystems.shooter.hood.*;

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
      channelConfig.pulseRange(1000, 1500, 2000); // Default PWM pulses recommended by REV
      config.apply(ChannelId.fromInt(i), channelConfig);

    
    }

    servoHub.setBankPulsePeriod(Bank.kBank0_2, 20000); //TODO set this

    servo.setPowered(true);

    // Enables "brake" mode on servos
    servo.setEnabled(true);


    return servoHub.configure(config, ResetMode.kResetSafeParameters);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.encoderConnected = encoder.isConnected();
    inputs.currentPulseWidth = servo.getPulseWidth();
    inputs.rawRotations = encoder.getPosition().getValueAsDouble(); // Convert degrees to rotations
    
    inputs.degrees = inputs.rawRotations * 360.0; // Convert rotations to degrees
    
    inputs.degrees = inputs.rawRotations * 360.0; // Convert rotations to degrees
    inputs.encoderRotationsPerInfo = encoder.getVelocity().getValueAsDouble();
    inputs.servoEnabled = servo.isEnabled(); // Assuming a threshold of 0.1A to determine if the servo is powered
    inputs.appliedVolts = servo.getCurrent(); // Get the voltage applied to the servo
  }
  
  @Override
  public void setEncoderPositionDEG(double degrees) {
    encoder.setPosition(degrees / 360.0);
  }

    @Override
  public void stopAt(double angle) {
   if ((inputs.rotations > angle + 0.10) || (inputs.rotations < (angle - 0.10))) { //I will make them constants dont attack if its about them not being constants
     servo.setPulseWidth(1500); // Move to hard stop
    servo.setPulseWidth((int)(1500 + (angle * 500))); // Assuming 1500 is the center position and 500 is the scaling factor
   }
  }

  @Override
  public void setServoVelocity(double pulseWidth) {
   int currentRequested = ((int)pulseWidth * 500);
   int velocity = 1500 + currentRequested ;
   servo.setPulseWidth(velocity);

  }

  @Override
  public void homingPulseWidth() {
    servo.setPulseWidth(2000);
  }

  
    

}
