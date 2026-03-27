package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class IntakeCommands {
  private static Command transitionToIntake(Intake intake, CommandXboxController controller) {
    return intake(intake)
        .andThen(
            Commands.run(
                    () -> {
                      controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
                    })
                .withTimeout(0.25)
                .finallyDo(
                    () -> {
                      controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                    }));
  }

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
        });
  }

  public static Command toggleIntake(Intake intake, CommandXboxController controller) {
    return new ConditionalCommand(
        transitionToIntake(intake, controller)
            .onlyIf(
                () ->
                    intake.getState() == IntakeState.IDLE
                        || intake.getState() == IntakeState.STARING_CONFIG),
        idle(intake),
        () -> intake.getState() != IntakeState.INTAKING);
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

  public static Command autoSmoosh(Intake intake) {
    return Commands.run(
            () -> {
              intake.setState(IntakeState.SMOOSH);
            })
        .onlyIf(() -> intake.hasExtended());
  }

  public static Command autoSmoosh(Intake intake, double delay, double timeout) {
    return new SequentialCommandGroup(
            new WaitCommand(delay),
            Commands.runOnce(
                () -> {
                  intake.setState(IntakeState.SMOOSH);
                }),
            new WaitCommand(timeout),
            toggleOff(intake))
        .onlyIf(() -> intake.hasExtended());
  }
}
