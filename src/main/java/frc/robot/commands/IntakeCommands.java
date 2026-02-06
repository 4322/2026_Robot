package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class IntakeCommands {

  public static Command setExtend(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.setState(IntakeState.EXTEND);
        });
  }

  public static Command setRetract(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.setState(IntakeState.RETRACT);
        });
  }

  public static Command setEject(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.setState(IntakeState.EJECT);
        });
  }

  public static Command setIdle(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.setState(IntakeState.IDLE);
        });
  }

  public static Command setIntaking(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.setState(IntakeState.INTAKING);
        });
  }
}
