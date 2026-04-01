package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.DelayQueue;
import java.util.concurrent.Delayed;
import java.util.concurrent.LinkedBlockingDeque;

import org.littletonrobotics.junction.Logger;

public class HoodIOSim implements HoodIO {
  private double velocity = 0;
  private double rotations = 0;
  private double position = 0;
  private double maxRPS = 0.2;
  private double degreePerSecond = 37;
  private double prevPosition = 0;
  public Queue<Producer> positionQueue = new LinkedList<>();

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.encoderConnected = true;
    inputs.encoderRotations = rotations; // Convert degrees to rotations
    inputs.hoodDegrees = position;
    inputs.encoderRPS = velocity;
    inputs.servoEnabled = true;
  }

  @Override
  public void setEncoderHomed() {
    this.rotations = 0;
    this.position = 0;
  }

  @Override
  public void simEstimatedPosition() {
   
  }

  @Override
  public void setPulseWidth(int pulseWidth) {
    // TODO
    Logger.recordOutput("Shooter/Hood/pulseWidth", pulseWidth);
  }

  public class Producer {
    private double offeredPosition;
    private double timeAdded;
    public Producer(double offeredPosition, double timeAdded) {
      this.offeredPosition = offeredPosition;
      this.timeAdded = timeAdded;
    }
    public void addPosition(double offeredPosition) {
      timeAdded = Timer.getFPGATimestamp();
      positionQueue.offer(new Producer(offeredPosition, timeAdded));
    }
   
  }


   public class Consumer {
    private double timerCurrentValue;
    private double requestedPosition;
    public Consumer(double timerCurrentValue, double requestedPosition) {
      this.timerCurrentValue = timerCurrentValue;
      this.requestedPosition = requestedPosition;
    }

    public void checkPosition() {
      if (positionQueue.isEmpty()) {
        return;
      }
      timerCurrentValue = (Timer.getFPGATimestamp() - positionQueue.peek().timeAdded);
      if (timerCurrentValue >= 0.120) {
        requestedPosition = positionQueue.poll().offeredPosition;
     }
    }

    public double getRequestedPosition() {
      return requestedPosition;
    }
  }


}
