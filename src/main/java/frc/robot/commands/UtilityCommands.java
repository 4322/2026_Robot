package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.function.DoubleSupplier;

public class UtilityCommands {

  private UtilityCommands() {}

  public static Command waitSupplier(DoubleSupplier waitSeconds) {
    return Commands.run(
        () -> {
          new WaitCommand(waitSeconds.getAsDouble());
        });
  }
}
