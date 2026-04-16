package frc.robot.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class DriveTest extends SequentialCommandGroup {

  public DriveTest(Drive drive, DriveCommands driveCommands) {
    setName("DriveTest");

    addCommands(
        new InstantCommand(() -> Logger.recordOutput("Tester/testStarted", true)),
        new InstantCommand(
            () ->
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        drive.getMaxLinearSpeedMetersPerSec(), 0, 0, Rotation2d.fromDegrees(0)))),
        new ParallelCommandGroup(
            driveCommands.TesterDrive(drive, "Drive Forward"), new WaitCommand(5)),
        new InstantCommand(
            () ->
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        -drive.getMaxLinearSpeedMetersPerSec(), 0, 0, Rotation2d.fromDegrees(0)))),
        new ParallelCommandGroup(
            driveCommands.TesterDrive(drive, "Drive BackWard"), new WaitCommand(5)),
        new InstantCommand(
            () ->
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        0, drive.getMaxLinearSpeedMetersPerSec(), 0, Rotation2d.fromDegrees(0)))),
        new InstantCommand(
            () ->
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        0, drive.getMaxLinearSpeedMetersPerSec(), 0, Rotation2d.fromDegrees(0)))),
        new ParallelCommandGroup(
            driveCommands.TesterDrive(drive, "Drive Left"), new WaitCommand(5)));
  }
}
