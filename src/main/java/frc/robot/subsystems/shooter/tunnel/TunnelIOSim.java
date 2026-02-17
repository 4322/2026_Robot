package frc.robot.subsystems.shooter.tunnel;

public class TunnelIOSim implements TunnelIO {
    private double rate = 0.2;
    private double mechanismRotationsPerSec = 0.0;
    private double targetVelocity = 0.0;

    @Override
    public void updateInputs(TunnelIOInputs inputs) {
        inputs.motorConnected = true;
        simVelocity();
        inputs.mechanismRotationsPerSec = mechanismRotationsPerSec;
    }

    @Override
    public void setTargetMechanismRotations(double velocity) {
        targetVelocity = velocity;
    }

    @Override
    public void stop() {
        targetVelocity = 0.0;
    }

    @Override
    public void enableBrakeMode(boolean enable) {
        return;
    }

    private void simVelocity() {
        if (mechanismRotationsPerSec < targetVelocity) {
            mechanismRotationsPerSec += rate;
        } else if (mechanismRotationsPerSec > targetVelocity) {
            mechanismRotationsPerSec -= rate;
        }
    }

}
