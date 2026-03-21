public class CrtTest {
    final static double CANCoderOneRatio = 90 / 10.0;
    final static double CANCoderTwoRatio = 90 / 19.0;

    private static double getRotation(double encoderOne, double encoderTwo) {
        // Based off of 4522's "brute force" solver:
        // https://www.chiefdelphi.com/uploads/short-url/vvrM1V1pqvDnnZfHtAhS02mBVIi.pdf

        // HACK: Look. I know this value works, okay?
        final int GCD = 90;
        final double LCM = lcm(CANCoderOneRatio, CANCoderTwoRatio);
        // Range of motion in rotations
        final double ROTATIONAL_RANGE = LCM / CANCoderOneRatio / CANCoderTwoRatio;

        //double encoderOne = inputs.encoderOneCount / Constants.Turret.CANCoderResolution;
        //double encoderTwo = inputs.encoderTwoCount / Constants.Turret.CANCoderResolution;

        double bestError = Double.MAX_VALUE;
        double turretFullRotations = 0.5;

        // look for the entry in both lists of possible values that is the same.
        for (int i = 0; i < GCD; i++) {
            // generate the candidate rotation value from encoder 1
            double candidate1 = mod((encoderOne + (double) i) / CANCoderOneRatio, ROTATIONAL_RANGE);
            for (int j = 0; j < GCD; j++) {
                // generate the candidate rotation value from encoder 2
                double candidate2 = mod(((encoderTwo + (double) j) / CANCoderTwoRatio), ROTATIONAL_RANGE);
                double error = Math.abs(candidate2 - candidate1);
                if (error < bestError) {
                bestError = error;
                turretFullRotations = candidate1;
                }
            }
        }
        return turretFullRotations;
  }

  private static double mod(double a, double b) {
    return ((a % b) + b) % b;
  }

  /**
   * Least common multiple that works on fractional values.
   * 
   * @param a value a
   * @param b value b
   * @return least common multiple
   */
  private static double lcm(double a, double b) {
    double i = 1.0;
    while (true) {
      if (a * i % b < 0.001) {
        return a * i;
      }
      i += 1.0;
    }
  }

  public static void main(String[] args) {
    for (int i = 0; i < 770; i++) {
        double encoderOne = (((double) i / 360.0) * CANCoderOneRatio * 4096) % 4096.0 / 4096.0;
        double encoderTwo = (((double) i / 360.0) * CANCoderTwoRatio * 4096) % 4096.0 / 4096.0;
        System.out.printf("deg: %d enc1: %f enc2: %f, reading: %f\n", i, encoderOne, encoderTwo, getRotation(encoderOne, encoderTwo) * 360.0);
    }
  }
}
