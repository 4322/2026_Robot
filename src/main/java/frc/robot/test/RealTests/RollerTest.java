package frc.robot.test.RealTests;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.rollers.Rollers;

import org.littletonrobotics.junction.Logger;

public class RollerTest extends SequentialCommandGroup {

  public RollerTest(Intake intake, Rollers rollers) {
    setName("RollerTest");

    addCommands(
        new InstantCommand(() -> Logger.recordOutput("Tester/testStarted", true)),
        new InstantCommand(
            () ->
                IntakeCommands.intake(intake)),
        new ParallelCommandGroup(
            IntakeCommands.rollerTesting(rollers, "string"), new WaitCommand(5))
        );
  }
}
