package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class HoodIOSim implements HoodIO {
  private double positionDeg = 0;
  private double requestedPositionDeg = 0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.encoderConnected = true;
    inputs.motorConnected = true;

    simPos();

    inputs.hoodDegrees = positionDeg;
    inputs.encoderDegrees = positionDeg / 5;
  }

  @Override
  public void setEncoderHomed() {
    this.positionDeg = 0;
    this.requestedPositionDeg = 0;
  }

  @Override
  public void setAngle(double requestedPosDeg) {
    this.requestedPositionDeg = requestedPosDeg;
  }

  private void simPos() {
    if (DriverStation.isEnabled()) {
      if (positionDeg < requestedPositionDeg) {
        positionDeg += (requestedPositionDeg - positionDeg) * Constants.Sim.hoodRate;
      } else {
        positionDeg -= (positionDeg - requestedPositionDeg) * Constants.Sim.hoodRate;
      }
    }
  }
}
