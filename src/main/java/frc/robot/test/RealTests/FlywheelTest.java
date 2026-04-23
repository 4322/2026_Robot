package frc.robot.test.RealTests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import org.littletonrobotics.junction.Logger;

public class FlywheelTest extends SequentialCommandGroup {

  public FlywheelTest(Flywheel flywheel, Intake intake, Shooter shooter, Drive drive) {
    setName("FlywheelTest");

    addCommands(
        new InstantCommand(() -> Logger.recordOutput("Tester/testStarted", true)),
        new InstantCommand(() -> IntakeCommands.intake(intake)),
        new InstantCommand(() -> IntakeCommands.idle(intake)),
        new InstantCommand(() -> ShooterCommands.aimAndShoot(shooter, drive, intake)),
        new ParallelCommandGroup(
            ShooterCommands.flywheelTesting(flywheel, "string"), new WaitCommand(5)));
  }
}
