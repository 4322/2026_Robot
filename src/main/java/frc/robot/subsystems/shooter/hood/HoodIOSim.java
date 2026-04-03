package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Sim;
import frc.robot.subsystems.Simulator;

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
  public int degrees;
  private double wantedPosition;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    simEstimatedPosition();
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



//Instead requets a degree value in sim with same pulse width method
  @Override
  public void setPulseWidth(int pulsewidth) {
    double pulseWidthToDegreeRatio = 0.9;
    this.degrees =
        MathUtil.clamp(
            Constants.Hood.homePulseWidth
                + (int)
                    (pulsewidth
                        * pulseWidthToDegreeRatio
                        * Constants.Hood.servoPositionScaleFactor),
            0,
            38);
      requestPosition(degrees);
  }

  public class Producer {
    private double offeredPosition;
    private double timeAdded;
    public Producer(double offeredPosition, double timeAdded) {
      this.offeredPosition = offeredPosition;
      this.timeAdded = timeAdded;
    }
  }

  public void requestPosition(double offeredPosition) {
    positionQueue.add(new Producer(offeredPosition, Simulator.getCurrentSimTime()));
  }

   public class Consumer {
    private double requestedPosition;
    public Consumer( double requestedPosition) {
      this.requestedPosition = requestedPosition;
    }

    public double getPosition() {
      if (positionQueue.isEmpty()) {
        return requestedPosition;
      }
      double timerCurrentValue = (Simulator.getCurrentSimTime() - positionQueue.peek().timeAdded);
      if (timerCurrentValue >= 0.120) {
      requestedPosition = positionQueue.poll().offeredPosition;
     }
     return requestedPosition;
    }

    
  }
  
  @Override
  public void simEstimatedPosition() {
    Consumer consumer = new Consumer(0);
    this.wantedPosition = consumer.getPosition();
    if (wantedPosition > this.position) {
      velocity = MathUtil.clamp(degreePerSecond, 0, maxRPS * 360);
      this.position += velocity * 0.02;
    } else if (wantedPosition < this.position) {
      velocity = MathUtil.clamp(-degreePerSecond, -maxRPS * 360, 0);
      this.position -= velocity * 0.02;
    } 
  }


}
