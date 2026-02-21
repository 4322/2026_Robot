package frc.robot.subsystems.vision.visionGlobalPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.visionGlobalPose.VisionGlobalPoseIO.GlobalPoseObservation;
import frc.robot.util.GeomUtil;
import frc.robot.util.PolynomialRegression;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class VisionGlobalPose extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionGlobalPoseIO[] io;
  private final VisionGlobalPoseIOInputsAutoLogged[] inputs;
  private final Drive drive;
  public int tagId;

  private PolynomialRegression xyStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.12, 0.14, 0.17, 0.27, 0.38},
          2);
  private PolynomialRegression thetaStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.049, 0.027, 0.059, 0.029, 0.068},
          1);

  public VisionGlobalPose(Drive drive, VisionGlobalPoseIO... io) {
    this.drive = drive;
    this.consumer = drive::addVisionMeasurement;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionGlobalPoseIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionGlobalPoseIOInputsAutoLogged();
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose2d> allRobotPoses = new LinkedList<>();
    List<Pose2d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose2d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Initialize logging values
      List<Pose2d> robotPoses = new LinkedList<>();
      List<Pose2d> robotPosesAccepted = new LinkedList<>();
      List<Pose2d> robotPosesRejected = new LinkedList<>();

      for (GlobalPoseObservation observation : inputs[cameraIndex].globalPoseObservations) {
        Pose3d disambiguatedRobotPose; // Robot pose chosen after disambiguation
        double avgTagDistance = observation.averageTagDistance();

        // Disambiguate to select a single robot pose
        if (observation.useMultiTag()) {
          disambiguatedRobotPose = observation.pose();
        } else {
          if (observation.ambiguity() < Constants.VisionGlobalPose.maxAmbiguity) {
            disambiguatedRobotPose = observation.pose();
          } else if (Math.abs(
                  observation
                      .pose()
                      .toPose2d()
                      .getRotation()
                      .minus(drive.getRotation())
                      .getRadians())
              < Math.abs(
                  observation
                      .altPose()
                      .toPose2d()
                      .getRotation()
                      .minus(drive.getRotation())
                      .getRadians())) {
            disambiguatedRobotPose = observation.pose();
          } else {
            disambiguatedRobotPose = observation.altPose();
            avgTagDistance = observation.averageTagDistanceAlt();
          }

          if (Constants.VisionGlobalPose.enableGlobalPoseTrigEstimation) {
            Pose2d visionRobotPose = disambiguatedRobotPose.toPose2d();
            Pose2d tagPos =
                FieldConstants.aprilTagFieldLayout
                    .getTagPose(inputs[cameraIndex].singleTagFiducialID)
                    .get()
                    .toPose2d();
            // Use gyro to correct for vision errors
            Rotation2d robotThetaError = drive.getRotation().minus(visionRobotPose.getRotation());

            // Account for rotation discontinuity from bound (-179,180]
            if (Math.abs(robotThetaError.getRadians()) > Math.PI) {
              double minThetaError =
                  robotThetaError.getDegrees() + (Math.signum(robotThetaError.getDegrees()) * -360);
              robotThetaError = Rotation2d.fromDegrees(minThetaError);
            }

            Pose2d tagToRobotPose = visionRobotPose.relativeTo(tagPos);
            visionRobotPose =
                tagPos.transformBy(
                    GeomUtil.poseToTransform(tagToRobotPose.rotateBy(robotThetaError)));

            disambiguatedRobotPose =
                new Pose3d(
                    new Translation3d(
                        visionRobotPose.getX(),
                        visionRobotPose.getY(),
                        disambiguatedRobotPose.getZ()),
                    new Rotation3d(visionRobotPose.getRotation()));

            Logger.recordOutput("Vision/TrigGlobalPose", disambiguatedRobotPose.toPose2d());
          }
        }

        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity()
                        > Constants.VisionGlobalPose.maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(disambiguatedRobotPose.getZ())
                    > Constants.VisionGlobalPose.maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || disambiguatedRobotPose.getX() < -0.5
                || disambiguatedRobotPose.getX() > FieldConstants.fieldLength + 0.5
                || disambiguatedRobotPose.getY() < -0.5
                || disambiguatedRobotPose.getY() > FieldConstants.fieldWidth + 0.5
                || avgTagDistance > Constants.VisionGlobalPose.maxAvgTagDistance;

        // Add pose to log
        robotPoses.add(disambiguatedRobotPose.toPose2d());
        if (rejectPose) {
          robotPosesRejected.add(disambiguatedRobotPose.toPose2d());
        } else {
          robotPosesAccepted.add(disambiguatedRobotPose.toPose2d());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double xyStdDev = 0.0;
        double thetaStdDev = 0.0;

        if (observation.useMultiTag()) {
          xyStdDev = Math.pow(avgTagDistance, 2.0) / observation.tagCount();
          thetaStdDev = Math.pow(avgTagDistance, 2.0) / observation.tagCount();

          consumer.accept(
              disambiguatedRobotPose.toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(
                  Constants.VisionGlobalPose.stdDevBaseline
                      * Constants.VisionGlobalPose.thetaStdDevBaseline
                      * xyStdDev,
                  Constants.VisionGlobalPose.stdDevBaseline
                      * Constants.VisionGlobalPose.thetaStdDevBaseline
                      * xyStdDev,
                  Constants.VisionGlobalPose.stdDevBaseline
                      * Constants.VisionGlobalPose.thetaStdDevBaseline
                      * thetaStdDev));
          Logger.recordOutput(
              "VisionGlobalPose/StdDev/XY",
              Constants.VisionGlobalPose.stdDevBaseline
                  * xyStdDev
                  * Constants.VisionGlobalPose.thetaStdDevBaseline);
          Logger.recordOutput(
              "VisionGlobalPose/StdDev/Theta",
              Constants.VisionGlobalPose.stdDevBaseline
                  * thetaStdDev
                  * Constants.VisionGlobalPose.thetaStdDevBaseline);
        } else {
          xyStdDev = Math.max(xyStdDevModel.predict(avgTagDistance), 0.000001);

          if (Constants.VisionGlobalPose.enableGlobalPoseTrigEstimation) {
            thetaStdDev = 4322.0;
          } else {
            thetaStdDev =
                Math.max(thetaStdDevModel.predict(avgTagDistance), 0.000001)
                    * Constants.VisionGlobalPose.stdDevBaseline;
          }

          consumer.accept(
              disambiguatedRobotPose.toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(
                  Constants.VisionGlobalPose.stdDevBaseline * xyStdDev,
                  Constants.VisionGlobalPose.stdDevBaseline * xyStdDev,
                  thetaStdDev));

          Logger.recordOutput(
              "VisionGlobalPose/StdDev/XY", Constants.VisionGlobalPose.stdDevBaseline * xyStdDev);
          Logger.recordOutput(
              "VisionGlobalPose/StdDev/Theta",
              Constants.VisionGlobalPose.stdDevBaseline * thetaStdDev);
        }
      }

      // Log camera datadata
      Logger.recordOutput(
          "VisionGlobalPose/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose2d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "VisionGlobalPose/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose2d[robotPosesRejected.size()]));
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose2d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose2d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose2d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public Pose2d getHybridPose(Drive drive) {
    return new Pose2d();
  }
}
