package frc.robot.test;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;

public class DriveTest extends SequentialCommandGroup{
    public DriveTest(Drive drive) {
        setName("DriveTest");

        addCommands(
        new InstantCommand(() -> Logger.recordOutput("Tester/testStarted", true)),
        new InstantCommand(() -> drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(drive.getMaxLinearSpeedMetersPerSec(), 0, 0, Rotation2d.fromDegrees(0)))),
        // new alongWith(
        //     new InstantCommand(() -> Logger.recordOutput("Tester/fieldRelativeVelocity", drive.getFieldRelativeVelocity().getVelocity().toString())),
            new WaitCommand(5),
        // ),
         new InstantCommand(() -> drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(drive.getMaxLinearSpeedMetersPerSec(), 0, 0, Rotation2d.fromDegrees(0)))),
        new WaitCommand(5),
        new InstantCommand(() -> drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(drive.getMaxLinearSpeedMetersPerSec(), 0, 0, Rotation2d.fromDegrees(0)))));
    }
    
}
