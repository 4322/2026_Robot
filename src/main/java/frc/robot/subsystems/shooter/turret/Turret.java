package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.util.ClockUtil;
import org.littletonrobotics.junction.Logger;

public class Turret {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private Double desiredDeg = 0.0;
  private boolean minInclusive = false;

  public enum turretState {
    DISABLED,
    SET_TURRET_ANGLE,
    UNWIND
  }

  public turretState state = turretState.DISABLED;

  public Turret(TurretIO io) {
    this.io = io;
    io.updateInputs(inputs);
    io.setPosition(getRotation()); // busted
    io.setPosition(0.5); // manual homing to the rear
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/State", state);
    Logger.recordOutput("Turret/needToUnwind", needsToUnwind());

    switch (Constants.turretMode) {
      case DISABLED -> {}
      case TUNING -> {}
      case NORMAL -> {
        switch (state) {
            /*
            case DISABLED -> {
              break;
            }
            case UNWIND -> {
              if (desiredDeg >= Constants.Turret.maxUnwindLimitDeg) {
                desiredDeg = (desiredDeg - 360);
              } else if (desiredDeg <= Constants.Turret.minUnwindLimitDeg) {
                desiredDeg = (desiredDeg + 360);
              } else {
                desiredDeg = 0.0;
              }
              if (!needsToUnwind()
                  && (isAtGoal() || (inputs.turretDegs >= -180 && inputs.turretDegs <= 180))) {
                state = turretState.SET_TURRET_ANGLE;
              }
            }
            case SET_TURRET_ANGLE -> {
              if (needsToUnwind()) {
                state = turretState.UNWIND;
              }
              if (desiredDeg != null) {
                io.setAngle(desiredDeg);
              }
            } */
        }
      }
    }
  }

  public void requestAngle(Double angle, boolean safeToUnwind) {
    this.desiredDeg = angle;
    Logger.recordOutput("Turret/desiredDeg", desiredDeg);
    if (Constants.turretLocked) {
      return;
    }
    double diffFromMid =
        Units.radiansToDegrees(
            MathUtil.angleModulus(
                Units.degreesToRadians(desiredDeg - Constants.Turret.midPointPhysicalDeg)));
    desiredDeg = Constants.Turret.midPointPhysicalDeg + diffFromMid;
    if (RobotContainer.intake.isExtended()) {
      io.setAngle(desiredDeg);
    }

    /*
    if (desiredDeg != null) {
      if (needsToUnwind()) {
        state = turretState.UNWIND;
      }
    }
    if (desiredDeg != null) {
      if (inputs.turretDegs + 180 >= Constants.Turret.maxPhysicalLimitDeg) {
        minInclusive = true;
      } else if (inputs.turretDegs - 180 <= Constants.Turret.minPhysicalLimitDeg) {
        minInclusive = false;
      }
      desiredDeg = angleDistance(desiredDeg, inputs.turretDegs, minInclusive);
      if (state != turretState.UNWIND && !needsToUnwind()) {
        state = turretState.SET_TURRET_ANGLE;
      }
    }
    if (safeToUnwind && needsToUnwind() || desiredDeg == null) {
      desiredDeg = Constants.Turret.midPointPhysicalDeg;
    }
      */
    Logger.recordOutput("Turret/adjustedDeg", desiredDeg);
  }

  public boolean needsToUnwind() {
    return false; /*
                  return (inputs.turretDegs >= Constants.Turret.maxUnwindLimitDeg
                      || inputs.turretDegs <= Constants.Turret.minUnwindLimitDeg);*/
  }

  public boolean isAtGoal() {
    if (Constants.turretLocked) {
      // desired turret angle is required robot heading when turret is locked
      return MathUtil.isNear(
          desiredDeg,
          RobotContainer.drive.getRotation().getDegrees(),
          Constants.Turret.goalToleranceLockedDeg);
    } else if (Constants.turretMode == Constants.SubsystemMode.DISABLED) {
      return true;
    } else {
      return MathUtil.isNear(desiredDeg, inputs.turretDegs, Constants.Turret.goalToleranceDeg);
    }
  }

  public void setTurretAngleState() {
    state = turretState.SET_TURRET_ANGLE;
  }

  public void unwind() {
    desiredDeg = setAngleWithinMidpoint();
    state = turretState.UNWIND;
  }

  public double getAngle() {
    if (Constants.turretLocked) {
      return 90;
    } else {
      return inputs.turretDegs;
    }
  }

  public void setBrakeMode(Boolean mode) {
    io.setBrakeMode(mode);
  }

  private double angleDistance(double targetAngle, double currentAngle, boolean minInclusive) {
    double angleDistance =
        ClockUtil.inputModulus(targetAngle - currentAngle, -180, 180, minInclusive) + currentAngle;
    return angleDistance;
  }

  private double setAngleWithinMidpoint() {
    desiredDeg = desiredDeg == null ? Constants.Turret.midPointPhysicalDeg : desiredDeg;
    if (desiredDeg == Constants.Turret.midPointPhysicalDeg) {
      return desiredDeg;
    }
    return desiredDeg =
        MathUtil.isNear(Constants.Turret.midPointPhysicalDeg, desiredDeg, 180)
            ? desiredDeg
            : Constants.Turret.midPointPhysicalDeg;
  }

  private double getRotation() {

    // Based off of 4522's "brute force" solver:
    // https://www.chiefdelphi.com/uploads/short-url/vvrM1V1pqvDnnZfHtAhS02mBVIi.pdf
    // This is only capable of calculating rotation in the 0-360 degree range.
    // This is because there's an ambiguity when the turret rotates 360 degrees; at one full
    // rotation both sensors will be at (0, 0) again.
    // Might be problematic in a brownout + over-rotated turret, but that's not here nor there.
    final int GCD = Constants.Turret.CANCoderOneRatio * Constants.Turret.CANCoderTwoRatio;
    double encoderOne = inputs.encoderOneCount / Constants.Turret.CANCoderResolution;
    double encoderTwo = inputs.encoderTwoCount / Constants.Turret.CANCoderResolution;

    double bestError = Double.MAX_VALUE;
    double turretFullRotations = 0.5;

    // look for the entry in both lists of possible values that is the same.
    for (int i = 0; i < GCD; i++) {
      // generate the candidate rotation value from encoder 1
      double candidate1 = mod(encoderOne + (double) i / Constants.Turret.CANCoderOneRatio, 1.0);
      for (int j = 0; j < GCD; j++) {
        // generate the candidate rotation value from encoder 2
        double candidate2 = mod((encoderTwo + (double) i / Constants.Turret.CANCoderTwoRatio), 1.0);
        double error = Math.abs(candidate2 - candidate1);
        if (error < bestError) {
          bestError = error;
          turretFullRotations = candidate1;
        }
      }
    }
    Logger.recordOutput("Turret/fullRotations", turretFullRotations);
    return turretFullRotations;

    // int CANCoderOneMod = mod(inputs.encoderOneCount, Constants.Turret.CANCoderTwoRatio);
    // int CANCoderTwoMod = mod(inputs.encoderTwoCount, Constants.Turret.CANCoderOneRatio);
    // double turretFullRotations = mod((10 * CANCoderOneMod) + (36 * CANCoderTwoMod), 45);
    // double turretFractionalRotations =
    //    inputs.encoderTwoCount
    //        / (double) Constants.Turret.CANCoderResolution
    //        / Constants.Turret.CANCoderTwoRatio;
    // Logger.recordOutput("Turret/fullRotations", turretFullRotations);
    // Logger.recordOutput("Turret/fractionalRotations", turretFractionalRotations);
    // return turretFullRotations + turretFractionalRotations;
  }

  private static double mod(double a, double b) {
    return ((a % b) + b) % b;
  }
    
    // Gear ratios
    private static final double GEAR_RATIO_5X = 5.0;
    private static final double GEAR_RATIO_9X = 9.0;
    private static final double MAX_TURRET_ROTATIONS = 45.0;
    
    // Cached turret position
    private double turretPositionRotations = 0.0;
    private boolean positionValid = false;
    
    // Tolerance for floating point comparison
    private static final double EPSILON = 1e-6;
    
    /**
     * Calculates the absolute turret position on power-up without moving.
     * Uses Chinese Remainder Theorem approach with the two geared encoders.
     */
    private void calculateInitialPosition() {
        double a = getAbsolutePositionNormalized(encoder5x);
        double b = getAbsolutePositionNormalized(encoder9x);
        
        turretPositionRotations = solveTurretPosition(a, b);
        positionValid = true;
    }
    
    /**
     * Gets normalized absolute position (0.0 to 1.0) from CANcoder.
     * Handles CTRE's continuous position reporting.
     */
    private double getAbsolutePositionNormalized(CANcoder encoder) {
        // getAbsolutePosition() returns rotations (can be negative or >1)
        // We want the fractional part 0.0 to 1.0
        double pos = encoder.getAbsolutePosition().getValueAsDouble();
        pos = pos % 1.0;
        if (pos < 0) pos += 1.0;
        return pos;
    }
    
    /**
     * Core algorithm: Solves for turret position given two encoder readings.
     * 
     * Math: Find P such that:
     *   (5 * P) % 1 = a
     *   (9 * P) % 1 = b
     *   0 <= P < 45
     */
    private double solveTurretPosition(double a, double b) {
        // We need integers i, j where:
        // 5*P = a + i  (i is which 5x rotation we're in)
        // 9*P = b + j  (j is which 9x rotation we're in)
        // 
        // Cross multiply: 9*(a + i) = 5*(b + j)
        // 9a + 9i = 5b + 5j
        // 9i - 5j = 5b - 9a
        
        double target = 5.0 * b - 9.0 * a;
        
        // Find integer solution to 9i - 5j = target
        // Extended Euclidean: 9*(-1) + 5*(2) = 1, so 9*(-target) - 5*(-2*target) = target
        // Wait, we need 9i - 5j = target, so i0 = -target, j0 = -2*target works:
        // 9*(-target) - 5*(-2*target) = -9t + 10t = t (but we want target, not -target)
        // Actually: 9*(target) - 5*(2*target) = 9t - 10t = -t
        // So: i0 = -target, j0 = -2*target gives 9*(-t) - 5*(-2t) = -9t + 10t = t
        
        // General solution: i = i0 + 5k, j = j0 + 9k for integer k
        
        double i0 = -target;
        double j0 = -2.0 * target;
        
        // Find k that puts i and j in valid ranges
        // i should be in [0, 5*45), j should be in [0, 9*45)
        
        double bestP = 0;
        double minError = Double.MAX_VALUE;
        boolean found = false;
        
        // Search reasonable range of k values
        for (int k = -50; k <= 50; k++) {
            double i = i0 + 5.0 * k;
            double j = j0 + 9.0 * k;
            
            // Check if i and j are valid integers (within epsilon)
            if (Math.abs(i - Math.round(i)) > EPSILON || Math.abs(j - Math.round(j)) > EPSILON) {
                continue;
            }
            
            int iInt = (int) Math.round(i);
            int jInt = (int) Math.round(j);
            
            // Check ranges
            if (iInt < 0 || iInt >= GEAR_RATIO_5X * MAX_TURRET_ROTATIONS) continue;
            if (jInt < 0 || jInt >= GEAR_RATIO_9X * MAX_TURRET_ROTATIONS) continue;
            
            // Calculate P from both equations
            double pFrom5 = (a + iInt) / GEAR_RATIO_5X;
            double pFrom9 = (b + jInt) / GEAR_RATIO_9X;
            
            double error = Math.abs(pFrom5 - pFrom9);
            
            if (error < minError && error < 0.01) { // Must agree within 1% of a rotation
                minError = error;
                bestP = (pFrom5 + pFrom9) / 2.0; // Average for stability
                found = true;
            }
        }
        
        if (!found) {
            // Fallback: try brute force search if analytical solution fails
            bestP = bruteForceSearch(a, b);
        }
        
        return bestP;
    }
    
    /**
     * Brute force search as fallback - checks all 45 possible rotations.
     */
    private double bruteForceSearch(double a, double b) {
        double bestP = 0;
        double minError = Double.MAX_VALUE;
        
        // Check at high resolution (0.001 rotation steps)
        for (double p = 0; p < MAX_TURRET_ROTATIONS; p += 0.001) {
            double expected5 = (GEAR_RATIO_5X * p) % 1.0;
            double expected9 = (GEAR_RATIO_9X * p) % 1.0;
            
            // Handle wrap-around for error calculation
            double err5 = Math.abs(expected5 - a);
            if (err5 > 0.5) err5 = 1.0 - err5;
            
            double err9 = Math.abs(expected9 - b);
            if (err9 > 0.5) err9 = 1.0 - err9;
            
            double totalError = err5 + err9;
            
            if (totalError < minError) {
                minError = totalError;
                bestP = p;
            }
        }
        
        return bestP;
    }
    
    /**
     * Returns the absolute turret position in rotations (0 to 45).
     * This is calculated on boot without requiring movement.
     */
    public double getTurretPositionRotations() {
        if (!positionValid) {
            calculateInitialPosition();
        }
        return turretPositionRotations;
    }
    
    /**
     * Returns turret position as Rotation2d (0 to 16200 degrees mapped to 0 to 2π).
     * Note: This loses the multi-rotation information! Use getTurretPositionRotations()
     * if you need to know which of the 45 rotations you're in.
     */
    public Rotation2d getTurretRotation2d() {
        // Rotation2d only handles 0-360, so we take fractional part
        double fraction = turretPositionRotations % 1.0;
        return Rotation2d.fromRotations(fraction);
    }
    
    /**
     * Get the continuous angle in degrees (0 to 16200).
     */
    public double getTurretAngleDegrees() {
        return turretPositionRotations * 360.0;
    }
    
    /**
     * Check if encoder readings are consistent (diagnostic).
     */
    public boolean isEncoderConsistent() {
        double a = getAbsolutePositionNormalized(encoder5x);
        double b = getAbsolutePositionNormalized(encoder9x);
        
        double calculatedP = solveTurretPosition(a, b);
        
        // Verify by computing what encoders should read
        double expected5 = (GEAR_RATIO_5X * calculatedP) % 1.0;
        double expected9 = (GEAR_RATIO_9X * calculatedP) % 1.0;
        
        double err5 = Math.abs(expected5 - a);
        if (err5 > 0.5) err5 = 1.0 - err5;
        
        double err9 = Math.abs(expected9 - b);
        if (err9 > 0.5) err9 = 1.0 - err9;
        
        return err5 < 0.01 && err9 < 0.01; // Within 1% of rotation
    }
}
}
