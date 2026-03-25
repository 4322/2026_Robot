package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class IntakeCommands {
  public static Command idle(Intake intake) {
    return Commands.runOnce(
            () -> {
              intake.setState(IntakeState.IDLE);
            })
        .onlyIf(() -> intake.hasExtended());
  }

  public static Command intake(Intake intake) {
    return Commands.runOnce(
            () -> {
              intake.setState(IntakeState.INTAKING);
            })
        .onlyIf(() -> intake.hasExtended());
  }

  public static Command toggleIntake(Intake intake, CommandXboxController controller) {
    return new ConditionalCommand(
        new ConditionalCommand(
            // Deploy if not extented
            Commands.runOnce(() -> intake.setState(IntakeState.DEPLOY)),
            Commands.runOnce(() -> intake.setState(IntakeState.INTAKING))
                .andThen(
                    Commands.run(
                            () -> {
                              controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
                            })
                        .withTimeout(0.25)
                        .finallyDo(
                            () -> {
                              controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                            })),
            () -> !intake.isExtended()),
        // Set to idle if not deploying or intaking
        Commands.runOnce(
                () -> {
                  intake.setState(IntakeState.IDLE);
                })
            .onlyIf(() -> intake.hasExtended()),
        () ->
            (!intake.hasExtended())
                || (intake.getState() != IntakeState.INTAKING && intake.hasExtended()));
  }

  public static Command eject(Intake intake) {
    return Commands.runOnce(
            () -> {
              intake.setState(IntakeState.EJECT);
            })
        .onlyIf(() -> intake.isExtended());
  }

  public static Command smoosh(Intake intake) {
    return Commands.runOnce(
            () -> {
              intake.setState(IntakeState.SMOOSH);
            })
        .onlyIf(() -> intake.hasExtended());
  }

  public static Command toggleOff(Intake intake) {
    return Commands.runOnce(
            () -> {
              if (intake.getPrevState() == Intake.IntakeState.INTAKING) {
                intake.setState(IntakeState.INTAKING);
              } else {
                intake.setState(IntakeState.IDLE);
              }
            })
        .onlyIf(() -> intake.hasExtended());
  }
}
