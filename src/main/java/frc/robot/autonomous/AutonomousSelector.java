package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.modes.DoNothing;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousSelector {

  private LoggedDashboardChooser<SequentialCommandGroup> autonomousSelector =
      new LoggedDashboardChooser<SequentialCommandGroup>("Autonomous");

  public enum AutoName {
    DO_NOTHING
  }

  private class Auto {
    AutoName name;
    SequentialCommandGroup command;

    private Auto(AutoName name, SequentialCommandGroup command) {
      this.name = name;
      this.command = command;
    }
  }

  private List<Auto> autos;
  private AutoName defaultAuto = AutoName.DO_NOTHING;

  public AutonomousSelector() {
    autos = List.of(new Auto(AutoName.DO_NOTHING, new DoNothing()));

    for (Auto nextAuto : autos) {
      if (nextAuto.name == defaultAuto) {
        autonomousSelector.addDefaultOption(nextAuto.name.toString(), nextAuto.command);
      } else {
        autonomousSelector.addOption(nextAuto.name.toString(), nextAuto.command);
      }
    }
  }

  public SequentialCommandGroup get() {
    if (Constants.currentMode == Mode.SIM) {
      for (Auto nextAuto : autos) {
        /* TODO
        if (nextAuto.name == Simulator.simulatedAuto) {
          return nextAuto.command;
        } */
      }
      // System.out.println("Simulated auto " + Simulator.simulatedAuto + " not found");
      System.exit(1);
    }
    return autonomousSelector.get();
  }
}
