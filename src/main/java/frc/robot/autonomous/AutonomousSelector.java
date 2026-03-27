package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autonomous.modes.CDepotOutpost;
import frc.robot.autonomous.modes.DoNothing;
import frc.robot.autonomous.modes.LHalfSweepShoot;
import frc.robot.autonomous.modes.LSweepBump;
import frc.robot.autonomous.modes.RDisruptSweepShoot;
import frc.robot.autonomous.modes.RFullSweepShoot;
import frc.robot.autonomous.modes.RHalfSuperSweepShoot;
import frc.robot.autonomous.modes.RHalfSweepShoot;
import frc.robot.autonomous.modes.RMidlineSweepShoot;
import frc.robot.autonomous.modes.RSweepBump;
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
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousSelector {

  private LoggedDashboardChooser<SequentialCommandGroup> autonomousSelector =
      new LoggedDashboardChooser<SequentialCommandGroup>("Autonomous");

  public enum AutoName {
    DO_NOTHING,
    C_DEPOT_OUTPOST,
    R_FULL_SWEEP_SHOOT,
    R_HALF_SWEEP_SHOOT,
    R_MIDLINE_SWEEP_SHOOT,
    R_DISRUPT_SWEEP_SHOOT,
    R_HALF_SUPER_SWEEP_SHOOT,
    L_HALF_SWEEP_SHOOT,

    R_2_SWEEP,
    L_2_SWEEP,
    R_SWEEP_BUMP,
    L_SWEEP_BUMP,
    R_ROUTPOST,

    DRIVE_WHEEL_RADIUS_CHARACTERIZATION,
    DRIVE_SIMPLE_FF_CHARACTERIZATION,
    DRIVE_SYS_ID_QUASISTATIC_FORWARD,
    DRIVE_SYS_ID_QUASISTATIC_REVERSE,
    DRIVE_SYS_ID_DYNAMIC_FORWARD,
    DRIVE_SYS_ID_DYNAMIC_REVERSE
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
            new Auto(AutoName.C_DEPOT_OUTPOST, new CDepotOutpost(drive, led, intake, shooter)),
            new Auto(AutoName.R_FULL_SWEEP_SHOOT, new RFullSweepShoot(drive, led, intake, shooter)),
            new Auto(
                AutoName.R_HALF_SWEEP_SHOOT,
                new RHalfSweepShoot(drive, led, intake, shooter, hood)),
            new Auto(
                AutoName.R_MIDLINE_SWEEP_SHOOT,
                new RMidlineSweepShoot(drive, led, intake, shooter)),
            new Auto(
                AutoName.R_DISRUPT_SWEEP_SHOOT,
                new RDisruptSweepShoot(drive, led, intake, shooter)),
            new Auto(AutoName.R_FULL_SWEEP_SHOOT, new RFullSweepShoot(drive, led, intake, shooter)),
            new Auto(
                AutoName.R_HALF_SUPER_SWEEP_SHOOT,
                new RHalfSuperSweepShoot(drive, led, intake, shooter)),
            new Auto(AutoName.R_FULL_SWEEP_SHOOT, new RFullSweepShoot(drive, led, intake, shooter)),
            new Auto(
                AutoName.L_HALF_SWEEP_SHOOT,
                new LHalfSweepShoot(drive, led, intake, shooter, hood)),
            new Auto(AutoName.L_SWEEP_BUMP, new LSweepBump(drive, led, intake, shooter)),
            new Auto(AutoName.R_SWEEP_BUMP, new RSweepBump(drive, led, intake, shooter)),
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
