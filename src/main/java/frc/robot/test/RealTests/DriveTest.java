package frc.robot.test.RealTests;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class DriveTest extends SequentialCommandGroup {

  public DriveTest(Drive drive) {
    setName("DriveTest");

    addCommands(
        new InstantCommand(() -> Logger.recordOutput("Tester/testStarted", true)),
        new ParallelCommandGroup(
                Commands.run(
                    () -> {
                      drive.runVelocity(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              drive.getMaxLinearSpeedMetersPerSec(),
                              0,
                              0,
                              Rotation2d.fromDegrees(0)));
                    },
                    drive),
                DriveCommands.DriveTesting(drive, "Drive Forward"))
            .withTimeout(5),
        new ParallelCommandGroup(
                Commands.run(
                    () -> {
                      drive.runVelocity(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              -drive.getMaxLinearSpeedMetersPerSec(),
                              0,
                              0,
                              Rotation2d.fromDegrees(0)));
                    },
                    drive),
                DriveCommands.DriveTesting(drive, "Drive Backard"))
            .withTimeout(5),
        new ParallelCommandGroup(
                Commands.run(
                    () -> {
                      drive.runVelocity(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              0,
                              drive.getMaxLinearSpeedMetersPerSec(),
                              0,
                              Rotation2d.fromDegrees(0)));
                    },
                    drive),
                DriveCommands.DriveTesting(drive, "Drive Left"))
            .withTimeout(5),
        new ParallelCommandGroup(
                Commands.run(
                    () -> {
                      drive.runVelocity(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              0,
                              -drive.getMaxLinearSpeedMetersPerSec(),
                              0,
                              Rotation2d.fromDegrees(0)));
                    },
                    drive),
                DriveCommands.DriveTesting(drive, "Drive Right"))
            .withTimeout(5),
        new ParallelCommandGroup(
                Commands.run(
                    () -> {
                      drive.runVelocity(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              0,
                              0,
                              drive.getMaxAngularSpeedRadPerSec(),
                              Rotation2d.fromDegrees(0)));
                    },
                    drive),
                DriveCommands.TurnTesting(drive, "Turn Right"))
            .withTimeout(5),
        new ParallelCommandGroup(
                Commands.run(
                    () -> {
                      drive.runVelocity(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              0,
                              0,
                              -drive.getMaxAngularSpeedRadPerSec(),
                              Rotation2d.fromDegrees(0)));
                    },
                    drive),
                DriveCommands.TurnTesting(drive, "Turn Left"))
            .withTimeout(5));
  }
}
