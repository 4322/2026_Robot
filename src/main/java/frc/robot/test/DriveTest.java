package frc.robot.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class DriveTest extends SequentialCommandGroup {
  Module[] modules;

  public DriveTest(Drive drive) {
    setName("DriveTest");

    addCommands(
        new InstantCommand(() -> Logger.recordOutput("Tester/testStarted", true)),
        new InstantCommand(
            () ->
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        drive.getMaxLinearSpeedMetersPerSec(), 0, 0, Rotation2d.fromDegrees(0)))),
        new WaitCommand(3),
        new InstantCommand(
            () ->
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        drive.getMaxLinearSpeedMetersPerSec(), 0, 0, Rotation2d.fromDegrees(0)))),
        new WaitCommand(5),
        new InstantCommand(
            () ->
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        drive.getMaxLinearSpeedMetersPerSec(), 0, 0, Rotation2d.fromDegrees(0)))));
  }
}
