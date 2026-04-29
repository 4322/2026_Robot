package frc.robot.test.RealTests;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.spindexer.Spindexer;

import org.littletonrobotics.junction.Logger;

public class SpindexerTest extends SequentialCommandGroup {

  public SpindexerTest(Spindexer spindexer, Intake intake, Shooter shooter, Drive drive) {
    setName("SpindexerTest");

    addCommands(
        new InstantCommand(() -> Logger.recordOutput("Tester/testStarted", true)),
        IntakeCommands.intake(intake),
        IntakeCommands.idle(intake),
        new ParallelCommandGroup(
            Commands.run(
                () -> {
                  spindexer.requestGoal(Constants.Spindexer.shootRPS);
                }),
            ShooterCommands.spindexerTesting(spindexer, "shoot Spindexer")));
  }
}
