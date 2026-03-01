package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import org.littletonrobotics.junction.Logger;

public class IntakeCommands {

  public static Command setRetract(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.setState(IntakeState.RETRACT);
        },
        intake);
  }

  public static Command setEject(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.setState(IntakeState.EJECT);
        },
        intake);
  }

  public static Command setIdle(Intake intake) {
    return Commands.run(
        () -> {
          Logger.recordOutput("Intake/Commands/setIdle", true);
          intake.setState(IntakeState.IDLE);
        },
        intake);
  }

  public static Command setIntaking(Intake intake) {
    return Commands.run(
            () -> {
              intake.setState(IntakeState.INTAKING);
              Logger.recordOutput("Intake/Commands/setIntaking", true);
            },
            intake)
        .andThen(
            new InstantCommand(() -> Logger.recordOutput("Intake/Commands/setIntaking", false)));
  }
}
