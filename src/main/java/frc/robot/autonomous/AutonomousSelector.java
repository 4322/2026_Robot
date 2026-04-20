package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autonomous.modes.CenterStartToDepot;
import frc.robot.autonomous.modes.DoNothing;
import frc.robot.autonomous.modes.L2056;
import frc.robot.autonomous.modes.L2Sweep;
import frc.robot.autonomous.modes.R2056;
import frc.robot.autonomous.modes.R2Sweep;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.subsystems.Simulator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetection;
import frc.robot.util.LoggedTunableNumber;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousSelector {

  private LoggedDashboardChooser<SequentialCommandGroup> autonomousSelector =
      new LoggedDashboardChooser<SequentialCommandGroup>("Autonomous");

  private LoggedDashboardChooser<AutoStartPosition> startPositionSelector =
      new LoggedDashboardChooser<AutoStartPosition>("Auto Start Position");

  private static LoggedTunableNumber autoStartDelay = new LoggedTunableNumber("Auto Start Delay");

  public enum AutoName {
    DO_NOTHING,
    C_DEPOT_OUTPOST,
    R_FULL_SWEEP_SHOOT,
    R_HALF_SWEEP_SHOOT,
    R_DISRUPT_SWEEP_SHOOT,
    R_HALF_SUPER_SWEEP_SHOOT,
    L_HALF_SWEEP_SHOOT,

    R_2_SWEEP,
    L_2_SWEEP,
    R_SWEEP_BUMP,
    L_SWEEP_BUMP,
    R_ROUTPOST,
    R_2056,
    L_2056,

    C_START_TO_DEPOT,

    DRIVE_WHEEL_RADIUS_CHARACTERIZATION,
    DRIVE_SIMPLE_FF_CHARACTERIZATION,
    DRIVE_SYS_ID_QUASISTATIC_FORWARD,
    DRIVE_SYS_ID_QUASISTATIC_REVERSE,
    DRIVE_SYS_ID_DYNAMIC_FORWARD,
    DRIVE_SYS_ID_DYNAMIC_REVERSE
  }

  public enum AutoStartPosition {
    INSIDE_TRENCH,
    OUTSIDE_TRENCH
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

  public AutonomousSelector(
      Drive drive,
      Hood hood,
      Turret turret,
      Shooter shooter,
      VisionObjectDetection visionObjectDetection,
      LED led,
      Intake intake) {
    autos =
        List.of(
            new Auto(AutoName.DO_NOTHING, new DoNothing(hood)),
            new Auto(
                AutoName.R_2_SWEEP,
                new R2Sweep(drive, intake, shooter, startPositionSelector, autoStartDelay)),
            new Auto(
                AutoName.L_2_SWEEP,
                new L2Sweep(drive, intake, shooter, startPositionSelector, autoStartDelay)),
            new Auto(
                AutoName.R_2056,
                new R2056(drive, intake, shooter, startPositionSelector, autoStartDelay)),
            new Auto(
                AutoName.L_2056,
                new L2056(drive, intake, shooter, startPositionSelector, autoStartDelay)),
            new Auto(
                AutoName.C_START_TO_DEPOT,
                new CenterStartToDepot(drive, intake, shooter, autoStartDelay)),
            new Auto(
                AutoName.DRIVE_WHEEL_RADIUS_CHARACTERIZATION,
                new SequentialCommandGroup(
                    Commands.race(
                        DriveCommands.wheelRadiusCharacterization(drive),
                        Commands.waitSeconds(60)))),
            new Auto(
                AutoName.DRIVE_SIMPLE_FF_CHARACTERIZATION,
                new SequentialCommandGroup(DriveCommands.feedforwardCharacterization(drive))),
            new Auto(
                AutoName.DRIVE_SYS_ID_QUASISTATIC_FORWARD,
                new SequentialCommandGroup(
                    Commands.race(
                        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
                        Commands.waitSeconds(6)))),
            new Auto(
                AutoName.DRIVE_SYS_ID_QUASISTATIC_REVERSE,
                new SequentialCommandGroup(
                    Commands.race(
                        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                        Commands.waitSeconds(6)))),
            new Auto(
                AutoName.DRIVE_SYS_ID_DYNAMIC_FORWARD,
                new SequentialCommandGroup(
                    Commands.race(
                        drive.sysIdDynamic(SysIdRoutine.Direction.kForward),
                        Commands.waitSeconds(6)))),
            new Auto(
                AutoName.DRIVE_SYS_ID_DYNAMIC_REVERSE,
                new SequentialCommandGroup(
                    Commands.race(
                        drive.sysIdDynamic(SysIdRoutine.Direction.kReverse),
                        Commands.waitSeconds(6)))));

    for (Auto nextAuto : autos) {
      if (nextAuto.name == defaultAuto) {
        autonomousSelector.addDefaultOption(nextAuto.name.toString(), nextAuto.command);
      } else {
        autonomousSelector.addOption(nextAuto.name.toString(), nextAuto.command);
      }
    }

    startPositionSelector.addDefaultOption("Inside Trench", AutoStartPosition.INSIDE_TRENCH);
    startPositionSelector.addOption("Outside Trench", AutoStartPosition.OUTSIDE_TRENCH);
  }

  public SequentialCommandGroup get() {
    if (Constants.currentMode == Mode.SIM) {
      for (Auto nextAuto : autos) {
        if (nextAuto.name == Simulator.getAutoScenario()) {
          Logger.recordOutput("AutoName", Simulator.getAutoScenario());
          return nextAuto.command;
        }
      }
      System.out.println("Simulated auto " + Simulator.getAutoScenario() + " not found");
      System.exit(1);
    }
    return autonomousSelector.get();
  }
}
