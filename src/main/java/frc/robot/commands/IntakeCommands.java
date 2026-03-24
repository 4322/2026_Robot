package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
  public static Command idle(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.requestIdle();
        });
  }

  public static Command intake(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.requestIntake();
        });
  }

  public static Command intake(Intake intake, CommandXboxController controller) {
    return Commands.runOnce(
            () -> {
              intake.requestIntake();
            })
        .andThen(
            Commands.run(
                    () -> {
                      controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
                    })
                .withTimeout(0.5)
                .finallyDo(
                    () -> {
                      controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                    }));
  }

  public static Command eject(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.requestEject();
        });
  }

  public static Command smoosh(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.requestSmoosh();
        });
  }
}
