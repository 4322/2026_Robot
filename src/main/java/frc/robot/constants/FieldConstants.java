package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class FieldConstants {
  public static double fieldLength = Units.inchesToMeters(651.22);
  public static double fieldWidth = Units.inchesToMeters(317.69);

  public static double centerLineX = fieldLength / 2;
  public static double centerLineY = fieldWidth / 2;
  public static double blueLineX = Units.inchesToMeters(182.11);
  public static double redLineX = Units.inchesToMeters(167.00);

  // All left/right designations are relative to blue alliance station
  public static class Blue {
    public static Translation2d hubTranslation = new Translation2d(blueLineX, centerLineY);
    public static Rectangle2d allianceZone =
        new Rectangle2d(
            new Translation2d(Units.inchesToMeters(156.61), Units.inchesToMeters(317.69)),
            new Translation2d(0, 0));
    public static Rectangle2d rightAllianceZone =
        new Rectangle2d(
            new Translation2d(0, 0),
            new Translation2d(blueLineX - Units.inchesToMeters(22.20), centerLineY));
    public static Rectangle2d leftAllianceZone =
        new Rectangle2d(
            new Translation2d(0, centerLineY),
            new Translation2d(blueLineX - Units.inchesToMeters(22.20), fieldWidth));
    public static Rectangle2d trenchLeft =
        new Rectangle2d(
            new Translation2d(
                blueLineX - Units.inchesToMeters(22.20),
                centerLineY + Units.inchesToMeters(133.47 - (24.97 + 12.00))),
            new Translation2d(blueLineX + Units.inchesToMeters(22.20), fieldWidth));
    public static Rectangle2d trenchRight =
        new Rectangle2d(
            new Translation2d(blueLineX - Units.inchesToMeters(22.20), 0),
            new Translation2d(
                blueLineX + Units.inchesToMeters(22.20), Units.inchesToMeters(50.59)));

    public static Rectangle2d bumpRight =
        new Rectangle2d(
            new Translation2d(
                blueLineX - Units.inchesToMeters(22.20), Units.inchesToMeters(50.59 + 12.00)),
            new Translation2d(
                blueLineX + Units.inchesToMeters(22.20),
                Units.inchesToMeters(50.59 + 12.00 + 73.00)));

    public static Rectangle2d bumpLeft =
        new Rectangle2d(
            new Translation2d(
                blueLineX - Units.inchesToMeters(22.20),
                Units.inchesToMeters(fieldWidth - (50.35 + 12.00 + 73.00))),
            new Translation2d(
                blueLineX + Units.inchesToMeters(22.20),
                Units.inchesToMeters(fieldWidth - (50.35 + 12.00))));

    public static Rectangle2d stopShootLeft =
        new Rectangle2d(
            new Translation2d(
                blueLineX - Units.inchesToMeters(22.20 + 6),
                centerLineY + Units.inchesToMeters(133.47 - (24.97 + 12.00))),
            new Translation2d(blueLineX + Units.inchesToMeters(22.20 + 6), fieldWidth));
    public static Rectangle2d stopShootRight =
        new Rectangle2d(
            new Translation2d(blueLineX - Units.inchesToMeters(22.20 + 6), 0),
            new Translation2d(
                blueLineX + Units.inchesToMeters(22.20 + 6), Units.inchesToMeters(50.59)));

    public static Rectangle2d frontOfHub =
        new Rectangle2d(
            new Translation2d(
                blueLineX - Units.inchesToMeters(22.20 + 40),
                centerLineY - Units.inchesToMeters(58.41 / 2)),
            new Translation2d(
                blueLineX - Units.inchesToMeters(22.20),
                centerLineY + Units.inchesToMeters(58.41 / 2)));
  }

  public static class Red {
    public static Translation2d hubTranslation = new Translation2d(redLineX, centerLineY);
    public static Rectangle2d allianceZone =
        new Rectangle2d(
            new Translation2d(Units.inchesToMeters(651.22), Units.inchesToMeters(317.69)),
            new Translation2d(Units.inchesToMeters(494.61), 0));

    public static Rectangle2d rightAllianceZone =
        new Rectangle2d(
            new Translation2d(redLineX + Units.inchesToMeters(22.20), 0),
            new Translation2d(fieldLength, centerLineY));
    public static Rectangle2d leftAllianceZone =
        new Rectangle2d(
            new Translation2d(redLineX + Units.inchesToMeters(22.20), centerLineY),
            new Translation2d(fieldLength, fieldWidth));

    public static Rectangle2d trenchLeft =
        new Rectangle2d(
            new Translation2d(
                redLineX - Units.inchesToMeters(22.20),
                centerLineY + Units.inchesToMeters(133.47 - (24.97 + 12.00))),
            new Translation2d(redLineX + Units.inchesToMeters(22.20), fieldWidth));
    public static Rectangle2d trenchRight =
        new Rectangle2d(
            new Translation2d(redLineX - Units.inchesToMeters(22.20), 0),
            new Translation2d(redLineX + Units.inchesToMeters(22.20), Units.inchesToMeters(50.59)));
    public static Rectangle2d bumpRight =
        new Rectangle2d(
            new Translation2d(
                redLineX - Units.inchesToMeters(22.20), Units.inchesToMeters(50.59 + 12.00)),
            new Translation2d(
                redLineX + Units.inchesToMeters(22.20),
                Units.inchesToMeters(50.59 + 12.00 + 73.00)));

    public static Rectangle2d bumpLeft =
        new Rectangle2d(
            new Translation2d(
                redLineX - Units.inchesToMeters(22.20),
                Units.inchesToMeters(fieldWidth - (50.35 + 12.00 + 73.00))),
            new Translation2d(
                redLineX + Units.inchesToMeters(22.20),
                Units.inchesToMeters(fieldWidth - (50.35 + 12.00))));

    public static Rectangle2d stopShootLeft =
        new Rectangle2d(
            new Translation2d(
                redLineX - Units.inchesToMeters(22.20 + 6),
                centerLineY + Units.inchesToMeters(133.47 - (24.97 + 12.00))),
            new Translation2d(redLineX + Units.inchesToMeters(22.20 + 6), fieldWidth));
    public static Rectangle2d stopShootRight =
        new Rectangle2d(
            new Translation2d(redLineX - Units.inchesToMeters(22.20 + 6), 0),
            new Translation2d(
                redLineX + Units.inchesToMeters(22.20 + 6), Units.inchesToMeters(50.59)));
    public static Rectangle2d frontOfHub =
        new Rectangle2d(
            new Translation2d(
                redLineX + Units.inchesToMeters(22.20),
                centerLineY - Units.inchesToMeters(58.41 / 2)),
            new Translation2d(
                redLineX + Units.inchesToMeters(22.20 + 40),
                centerLineY + Units.inchesToMeters(58.41 / 2)));
  }

  public static class Neutral {
    public static Rectangle2d rightNeutral =
        new Rectangle2d(new Translation2d(blueLineX, 0), new Translation2d(redLineX, centerLineY));
    public static Rectangle2d leftNeutral =
        new Rectangle2d(
            new Translation2d(blueLineX, centerLineX), new Translation2d(redLineX, fieldWidth));
    public static Rectangle2d backBlueHub =
        new Rectangle2d(
            new Translation2d(
                blueLineX + Units.inchesToMeters(22.20),
                centerLineY - Units.inchesToMeters(58.41 / 2)),
            new Translation2d(
                blueLineX + Units.inchesToMeters(80.00),
                centerLineY + Units.inchesToMeters(58.41 / 2)));
    public static Rectangle2d backRedHub =
        new Rectangle2d(
            new Translation2d(
                redLineX - Units.inchesToMeters(22.20),
                centerLineY - Units.inchesToMeters(58.41 / 2)),
            new Translation2d(
                redLineX - Units.inchesToMeters(80.00),
                centerLineY + Units.inchesToMeters(58.41 / 2)));
  }

  public static final double aprilTagWidth = Units.inchesToMeters(6.50);
  public static final int aprilTagCount = 32;
  public static final AprilTagFieldLayout aprilTagFieldLayout =
      new AprilTagFieldLayout(
          List.of(
              new AprilTag(
                  1,
                  new Pose3d(
                      11.8779798,
                      7.4247756,
                      0.889,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new AprilTag(
                  2,
                  new Pose3d(
                      11.9154194,
                      4.63804,
                      1.12395,
                      new Rotation3d(
                          new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476)))),
              new AprilTag(
                  3,
                  new Pose3d(
                      11.3118646,
                      4.3902376,
                      1.12395,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new AprilTag(
                  4,
                  new Pose3d(
                      11.3118646,
                      4.0346376,
                      1.12395,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new AprilTag(
                  5,
                  new Pose3d(
                      11.9154194,
                      3.4312352,
                      1.12395,
                      new Rotation3d(
                          new Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476)))),
              new AprilTag(
                  6,
                  new Pose3d(
                      11.8779798,
                      0.6444996,
                      0.889,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new AprilTag(
                  7,
                  new Pose3d(
                      11.9528844,
                      0.6444996,
                      0.889,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new AprilTag(
                  8,
                  new Pose3d(
                      12.2710194,
                      3.4312352,
                      1.12395,
                      new Rotation3d(
                          new Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476)))),
              new AprilTag(
                  9,
                  new Pose3d(
                      12.5191774,
                      3.6790376,
                      1.12395,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new AprilTag(
                  10,
                  new Pose3d(
                      12.5191774,
                      4.0346376,
                      1.12395,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new AprilTag(
                  11,
                  new Pose3d(
                      12.2710194,
                      4.63804,
                      1.12395,
                      new Rotation3d(
                          new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476)))),
              new AprilTag(
                  12,
                  new Pose3d(
                      11.9528844,
                      7.4247756,
                      0.889,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new AprilTag(
                  13,
                  new Pose3d(
                      16.5333172,
                      7.4033126,
                      0.55245,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new AprilTag(
                  14,
                  new Pose3d(
                      16.5333172,
                      6.9715126,
                      0.55245,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new AprilTag(
                  15,
                  new Pose3d(
                      16.5329616,
                      4.3235626,
                      0.55245,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new AprilTag(
                  16,
                  new Pose3d(
                      16.5329616,
                      3.8917626,
                      0.55245,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new AprilTag(
                  17,
                  new Pose3d(
                      4.6630844,
                      0.6444996,
                      0.889,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new AprilTag(
                  18,
                  new Pose3d(
                      4.6256194,
                      3.4312352,
                      1.12395,
                      new Rotation3d(
                          new Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476)))),
              new AprilTag(
                  19,
                  new Pose3d(
                      5.2291742,
                      3.6790376,
                      1.12395,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new AprilTag(
                  20,
                  new Pose3d(
                      5.2291742,
                      4.0346376,
                      1.12395,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new AprilTag(
                  21,
                  new Pose3d(
                      4.6256194,
                      4.63804,
                      1.12395,
                      new Rotation3d(
                          new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476)))),
              new AprilTag(
                  22,
                  new Pose3d(
                      4.6630844,
                      7.4247756,
                      0.889,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new AprilTag(
                  23,
                  new Pose3d(
                      4.5881798,
                      7.4247756,
                      0.889,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new AprilTag(
                  24,
                  new Pose3d(
                      4.2700194,
                      4.63804,
                      1.12395,
                      new Rotation3d(
                          new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476)))),
              new AprilTag(
                  25,
                  new Pose3d(
                      4.0218614,
                      4.3902376,
                      1.12395,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new AprilTag(
                  26,
                  new Pose3d(
                      4.0218614,
                      4.0346376,
                      1.12395,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new AprilTag(
                  27,
                  new Pose3d(
                      4.2700194,
                      3.4312352,
                      1.12395,
                      new Rotation3d(
                          new Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476)))),
              new AprilTag(
                  28,
                  new Pose3d(
                      4.5881798,
                      0.6444996,
                      0.889,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new AprilTag(
                  29,
                  new Pose3d(
                      0.007747,
                      0.6659626,
                      0.55245,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new AprilTag(
                  30,
                  new Pose3d(
                      0.007747,
                      1.0977626,
                      0.55245,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new AprilTag(
                  31,
                  new Pose3d(
                      0.0080772,
                      3.7457126,
                      0.55245,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new AprilTag(
                  32,
                  new Pose3d(
                      0.0080772,
                      4.1775126,
                      0.55245,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))))),
          fieldLength,
          fieldWidth);
}
