package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import frc.robot.util.HubShiftUtil;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.Optional;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private Timer allianceUpdateTimer = new Timer();
  private DigitalInput coastButton = new DigitalInput(Constants.dioCoastButton);
  private Timer coastButtonTimer = new Timer();

  public static Alliance alliance = DriverStation.Alliance.Blue;

  public static PathPlannerPath R_NeutralR_Intake_Mid_Flip;
  public static PathPlannerPath R_NeutralRMid_To_ShootR;
  public static PathPlannerPath R_NeutralR_Intake_To_Mid;
  public static PathPlannerPath R_Neutral_Mid_To_ShootR;
  public static PathPlannerPath R_NeutralR_Intake_Full_Flip;
  public static PathPlannerPath R_NeutralR_Intake_Full;
  public static PathPlannerPath R_StartR_To_NeutralR_Intake;
  public static PathPlannerPath R_NeutralR_Intake_Full_Midline;
  public static PathPlannerPath R_NeutralR_Intake_Full_Midline_Flip;
  public static PathPlannerPath R_StartR_To_NeutralR_Intake_Midline;
  public static PathPlannerPath R_StartR_To_NeutralR_Intake_Disrupt;
  public static PathPlannerPath R_NeutralR_Intake_Full_Disrupt;
  public static PathPlannerPath R_NeutralR_Intake_Full_Disrupt_Flip;

  public static PathPlannerPath L_NeutralL_Intake_Mid_Flip;
  public static PathPlannerPath L_NeutralLMid_To_ShootL;
  public static PathPlannerPath L_NeutralL_Intake_To_Mid;
  public static PathPlannerPath L_Neutral_Mid_To_ShootL;
  public static PathPlannerPath L_NeutralL_Intake_Full_Flip;
  public static PathPlannerPath L_NeutralL_Intake_Full;
  public static PathPlannerPath L_StartL_To_NeutralL_Intake;
  public static PathPlannerPath L_NeutralL_Intake_Full_Midline;
  public static PathPlannerPath L_NeutralL_Intake_Full_Midline_Flip;
  public static PathPlannerPath L_StartL_To_NeutralL_Intake_Midline;
  public static PathPlannerPath L_StartL_To_NeutralL_Intake_Disrupt;
  public static PathPlannerPath L_NeutralL_Intake_Full_Disrupt;
  public static PathPlannerPath L_NeutralL_Intake_Full_Disrupt_Flip;

  public static PathPlannerPath L_NeutralLMid_To_ShootL_LT;
  public static PathPlannerPath L_Neutral_Mid_To_ShootL_LT;

  public static PathPlannerPath R_NeutralRMid_To_ShootR_LT;
  public static PathPlannerPath R_Neutral_Mid_To_ShootR_LT;

  public static PathPlannerPath C_Depot_To_Outpost;
  public static PathPlannerPath C_Start_To_Depot;

  public static PathPlannerPath C_To_Depot;

  public static PathPlannerPath R_Half_SuperSweep_B;
  public static PathPlannerPath R_Half_SuperSweep_C;
  public static PathPlannerPath R_Half_SuperSweep_D;
  public static PathPlannerPath R_Half_SuperSweep_E;
  public static PathPlannerPath R_Half_SuperSweep_F;

  public static PathPlannerPath R_2SWEEP_A;
  public static PathPlannerPath R_2SWEEP_B;
  public static PathPlannerPath R_2SWEEP_CG;
  public static PathPlannerPath R_2SWEEP_DE;
  public static PathPlannerPath R_2SWEEP_F;
  public static PathPlannerPath R_2SWEEP_H;

  public static PathPlannerPath L_2SWEEP_A;
  public static PathPlannerPath L_2SWEEP_B;
  public static PathPlannerPath L_2SWEEP_CG;
  public static PathPlannerPath L_2SWEEP_DE;
  public static PathPlannerPath L_2SWEEP_F;
  public static PathPlannerPath L_2SWEEP_H;

  public static PathPlannerPath R_ROutpost_A;
  public static PathPlannerPath R_ROutpost_B;

  public static PathPlannerPath R_2056_A;
  public static PathPlannerPath R_2056_B;
  public static PathPlannerPath R_2056_C;
  public static PathPlannerPath R_2056_D;
  public static PathPlannerPath R_2056_G;

  public static PathPlannerPath L_2056_A;
  public static PathPlannerPath L_2056_B;
  public static PathPlannerPath L_2056_C;
  public static PathPlannerPath L_2056_D;
  public static PathPlannerPath L_2056_G;

  public Robot() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME); // Set a metadata value
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (Constants.currentMode) {
      case REAL:
        var directory = new File(Constants.logPath);
        if (!directory.exists()) {
          directory.mkdir();
        }
        var files = directory.listFiles();

        // delete all garbage hoot files and wpilogs not connected to ds before good wpilogs
        if (files != null) {
          for (File file : files) {
            if (file.getName().endsWith(".hoot")
                || (!file.getName().contains("-") && file.getName().endsWith(".wpilog"))) {
              file.delete();
              DriverStation.reportWarning("Deleted " + file.getName() + " to free up space", false);
            }
          }
        }

        // ensure that there is enough space on the roboRIO to log data
        // delete Redux logs first
        if (directory.getFreeSpace() < Constants.minFreeSpace) {
          files = directory.listFiles();
          if (files != null) {
            // Sorting the files by name will ensure that the oldest files are deleted first
            files = Arrays.stream(files).sorted().toArray(File[]::new);
            long bytesToDelete = Constants.minFreeSpace - directory.getFreeSpace();

            for (File file : files) {
              if (file.getName().endsWith(".rdxlog")) {
                try {
                  bytesToDelete -= Files.size(file.toPath());
                } catch (IOException e) {
                  DriverStation.reportError("Failed to get size of file " + file.getName(), false);
                  continue;
                }
                if (file.delete()) {
                  DriverStation.reportWarning(
                      "Deleted " + file.getName() + " to free up space", false);
                } else {
                  DriverStation.reportError("Failed to delete " + file.getName(), false);
                }
                if (bytesToDelete <= 0) {
                  break;
                }
              }
            }
          }
        }

        // delete akit logs if we still don't have enough space
        if (directory.getFreeSpace() < Constants.minFreeSpace) {
          files = directory.listFiles();
          if (files != null) {
            // Sorting the files by name will ensure that the oldest files are deleted first
            files = Arrays.stream(files).sorted().toArray(File[]::new);
            long bytesToDelete = Constants.minFreeSpace - directory.getFreeSpace();

            for (File file : files) {
              if (file.getName().endsWith(".wpilog")) {
                try {
                  bytesToDelete -= Files.size(file.toPath());
                } catch (IOException e) {
                  DriverStation.reportError("Failed to get size of file " + file.getName(), false);
                  continue;
                }
                if (file.delete()) {
                  DriverStation.reportWarning(
                      "Deleted " + file.getName() + " to free up space", false);
                } else {
                  DriverStation.reportError("Failed to delete " + file.getName(), false);
                }
                if (bytesToDelete <= 0) {
                  break;
                }
              }
            }
          }
        }

        Logger.addDataReceiver(
            new WPILOGWriter(Constants.logPath)); // Log to a USB stick is ("/U/logs")
        Logger.addDataReceiver(new RLOGServer());

        RobotController.setBrownoutVoltage(Constants.brownoutVoltage);

        break;
      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new RLOGServer());
        Logger.addDataReceiver(new WPILOGWriter("..")); // keep logs out of the repository
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        break;
    }

    Logger.start();
    Logger.disableConsoleCapture();

    try {
      C_Depot_To_Outpost = PathPlannerPath.fromPathFile("C_Depot_To_Outpost");
      C_Start_To_Depot = PathPlannerPath.fromPathFile("C_Start_To_Depot");

      R_NeutralR_Intake_Mid_Flip = PathPlannerPath.fromPathFile("R_NeutralR_Intake_Mid_Flip");
      R_NeutralRMid_To_ShootR = PathPlannerPath.fromPathFile("R_NeutralRMid_To_ShootR");
      R_NeutralR_Intake_To_Mid = PathPlannerPath.fromPathFile("R_NeutralR_Intake_To_Mid");
      R_Neutral_Mid_To_ShootR = PathPlannerPath.fromPathFile("R_Neutral_Mid_To_ShootR");
      R_NeutralR_Intake_Full_Flip = PathPlannerPath.fromPathFile("R_NeutralR_Intake_Full_Flip");
      R_NeutralR_Intake_Full = PathPlannerPath.fromPathFile("R_NeutralR_Intake_Full");
      R_StartR_To_NeutralR_Intake = PathPlannerPath.fromPathFile("R_StartR_To_NeutralR_Intake");

      R_NeutralR_Intake_Full_Midline =
          PathPlannerPath.fromPathFile("R_NeutralR_Intake_Full_Midline");
      R_NeutralR_Intake_Full_Midline_Flip =
          PathPlannerPath.fromPathFile("R_NeutralR_Intake_Full_Midline_Flip");
      R_StartR_To_NeutralR_Intake_Midline =
          PathPlannerPath.fromPathFile("R_StartR_To_NeutralR_Intake_Midline");
      R_StartR_To_NeutralR_Intake_Disrupt =
          PathPlannerPath.fromPathFile("R_StartR_To_NeutralR_Intake_Disrupt");
      R_NeutralR_Intake_Full_Disrupt =
          PathPlannerPath.fromPathFile("R_NeutralR_Intake_Full_Disrupt");
      R_NeutralR_Intake_Full_Disrupt_Flip =
          PathPlannerPath.fromPathFile("R_NeutralR_Intake_Full_Disrupt_Flip");

      R_NeutralRMid_To_ShootR_LT = PathPlannerPath.fromPathFile("R_NeutralRMid_To_ShootR_LT");
      R_Neutral_Mid_To_ShootR_LT = PathPlannerPath.fromPathFile("R_Neutral_Mid_To_ShootR_LT");

      R_Half_SuperSweep_B = PathPlannerPath.fromPathFile("R_Half_SuperSweep_B");
      R_Half_SuperSweep_C = PathPlannerPath.fromPathFile("R_Half_SuperSweep_C");
      R_Half_SuperSweep_D = PathPlannerPath.fromPathFile("R_Half_SuperSweep_D");
      R_Half_SuperSweep_E = PathPlannerPath.fromPathFile("R_Half_SuperSweep_E");
      R_Half_SuperSweep_F = PathPlannerPath.fromPathFile("R_Half_SuperSweep_F");

      R_2SWEEP_A = PathPlannerPath.fromPathFile("R_2Sweep_A");
      R_2SWEEP_B = PathPlannerPath.fromPathFile("R_2Sweep_B");
      R_2SWEEP_CG = PathPlannerPath.fromPathFile("R_2Sweep_CG");
      R_2SWEEP_DE = PathPlannerPath.fromPathFile("R_2Sweep_DE");
      R_2SWEEP_F = PathPlannerPath.fromPathFile("R_2Sweep_F");
      R_2SWEEP_H = PathPlannerPath.fromPathFile("R_2Sweep_H");

      R_ROutpost_A = PathPlannerPath.fromPathFile("R_ROutpost_A");
      R_ROutpost_B = PathPlannerPath.fromPathFile("R_ROutpost_B");

      R_2056_A = PathPlannerPath.fromPathFile("R_2056_A");
      R_2056_B = PathPlannerPath.fromPathFile("R_2056_B");
      R_2056_C = PathPlannerPath.fromPathFile("R_2056_C");
      R_2056_D = PathPlannerPath.fromPathFile("R_2056_D");
      R_2056_G = PathPlannerPath.fromPathFile("R_2056_G");

      C_To_Depot = PathPlannerPath.fromPathFile("Start_To_Depot");

      L_NeutralL_Intake_Mid_Flip = R_NeutralR_Intake_Mid_Flip.mirrorPath();
      L_NeutralLMid_To_ShootL = R_NeutralRMid_To_ShootR.mirrorPath();
      L_NeutralL_Intake_To_Mid = R_NeutralR_Intake_To_Mid.mirrorPath();
      L_Neutral_Mid_To_ShootL = R_Neutral_Mid_To_ShootR.mirrorPath();
      L_NeutralL_Intake_Full_Flip = R_NeutralR_Intake_Full_Flip.mirrorPath();
      L_NeutralL_Intake_Full = R_NeutralR_Intake_Full.mirrorPath();
      L_StartL_To_NeutralL_Intake = R_StartR_To_NeutralR_Intake.mirrorPath();
      L_NeutralL_Intake_Full_Midline = R_NeutralR_Intake_Full_Midline.mirrorPath();
      L_NeutralL_Intake_Full_Midline_Flip = R_NeutralR_Intake_Full_Midline_Flip.mirrorPath();
      L_StartL_To_NeutralL_Intake_Midline = R_StartR_To_NeutralR_Intake_Midline.mirrorPath();
      L_StartL_To_NeutralL_Intake_Disrupt = R_StartR_To_NeutralR_Intake_Disrupt.mirrorPath();
      L_NeutralL_Intake_Full_Disrupt = R_NeutralR_Intake_Full_Disrupt.mirrorPath();
      L_NeutralL_Intake_Full_Disrupt_Flip = R_NeutralR_Intake_Full_Disrupt_Flip.mirrorPath();

      L_NeutralLMid_To_ShootL_LT = R_NeutralRMid_To_ShootR_LT.mirrorPath();
      L_Neutral_Mid_To_ShootL_LT = R_Neutral_Mid_To_ShootR_LT.mirrorPath();

      L_2SWEEP_A = R_2SWEEP_A.mirrorPath();
      L_2SWEEP_B = R_2SWEEP_B.mirrorPath();
      L_2SWEEP_CG = R_2SWEEP_CG.mirrorPath();
      L_2SWEEP_DE = R_2SWEEP_DE.mirrorPath();
      L_2SWEEP_F = R_2SWEEP_F.mirrorPath();
      L_2SWEEP_H = R_2SWEEP_H.mirrorPath();

      L_2056_A = R_2056_A.mirrorPath();
      L_2056_B = R_2056_B.mirrorPath();
      L_2056_C = R_2056_C.mirrorPath();
      L_2056_D = R_2056_D.mirrorPath();
      L_2056_G = R_2056_G.mirrorPath();

    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner path - " + e.getMessage(), true);
      System.exit(1);
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    robotContainer.configureAutonomousSelector();

    allianceUpdateTimer.start();

    if (Constants.currentMode == Constants.Mode.SIM
        || Constants.currentMode == Constants.Mode.REPLAY) {
      // enable subsystems in sim mode
    }
  }

  @Override
  public void robotInit() {
    StatusLogger.disableAutoLogging(); // disable REV logging
    SignalLogger.stop(); // disable CTRE logging
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {

    /* roboRIO settings to control Java memory usage.

      If running out of memory, especially on a roboRIO 1, use:
      echo "vm.overcommit_memory=1" >> /etc/sysctl.conf
      echo "vm.vfs_cache_pressure=1000" >> /etc/sysctl.conf
      echo "vm.swappiness=100" >> /etc/sysctl.conf
      sync
      power cycle the RIO

      If CPU usage is a bigger concern than memory, use these values:
      echo "vm.overcommit_memory=2" >> /etc/sysctl.conf
      echo "vm.vfs_cache_pressure=50" >> /etc/sysctl.conf
      echo "vm.swappiness=30" >> /etc/sysctl.conf
      sync
      power cycle the RIO

      To restiore the default settings, edit /etc/sysctl.conf to set the
      following values:
        vm.overcommit_memory=2
        vm.vfs_cache_pressure=100
        vm.swappiness=60
        sync
        power cycle the RIO

      To stop the web server to save memory:
      /etc/init.d/systemWebServer stop; update-rc.d -f systemWebServer remove; sync
      chmod a-x /usr/local/natinst/etc/init.d/systemWebServer; sync

      To restart the web server in order to image a roboRIO 1:
      chmod a+x /usr/local/natinst/etc/init.d/systemWebServer; sync
      power cycle the RIO
    */

    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details).
    // Don't do this when there are issues causing the CPU to be maxed out for any reason.
    if (Constants.realTimeCommandScheduler) {
      Threads.setCurrentThreadPriority(true, 99);
    }

    // insure that the sim runs before everything else
    if (RobotContainer.simulator != null) {
      RobotContainer.simulator.periodic();
    }

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    RobotContainer.shooter.outputsPeriodic();

    RobotContainer.intake.periodicOutputs();

    // Return to non-RT thread priority (do not modify the first argument)
    if (Constants.realTimeCommandScheduler) {
      Threads.setCurrentThreadPriority(false, 10);
    }

    if (allianceUpdateTimer.hasElapsed(1)) {
      Optional<Alliance> allianceOptional = DriverStation.getAlliance();
      if (allianceOptional.isPresent()) {
        alliance = allianceOptional.get();
      }
      allianceUpdateTimer.restart();
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    if (Constants.currentMode != Constants.Mode.SIM) {
      if (!coastButton.get()) {
        // button is pressed in
        DriverStation.reportWarning("Activating Coast Mode", false);
        coastButtonTimer.start();
      }
      if (coastButtonTimer.hasElapsed(0.1)) {
        robotContainer.setBrakeMode(false);
      }
      if (coastButtonTimer.hasElapsed(10)) {
        DriverStation.reportWarning("Activating Brake Mode", false);
        robotContainer.setBrakeMode(true);
        coastButtonTimer.stop();
        coastButtonTimer.reset();
      }
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
      Logger.recordOutput("AutoName", autonomousCommand.getName());
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    if (Constants.hubShiftUtilenabled) {
      if (Constants.currentMode == Constants.Mode.SIM && Constants.hubShiftUtilWinOverride) {
        HubShiftUtil.setOverrideAlliance(Robot.alliance);
      }
      HubShiftUtil.initialize();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (Constants.hubShiftUtilenabled) {
      SmartDashboard.putNumber(
          "HubShiftUtil/Time Left in Shift", HubShiftUtil.getOfficialShiftInfo().remainingTime());
      SmartDashboard.putBoolean(
          "HubShiftUtil/Shift Active", HubShiftUtil.getOfficialShiftInfo().active());

      Logger.recordOutput("HubShiftUtil/schedule", HubShiftUtil.getSchedule());
      Logger.recordOutput("HubShiftUtil/fiveSecondsLeft", HubShiftUtil.fiveSecondsLeft());

      Logger.recordOutput(
          "HubShiftUtil/firstActiveAlliance", HubShiftUtil.getFirstActiveAlliance());
      Logger.recordOutput(
          "HubShiftUtil/officialShiftInfo/currentShift",
          HubShiftUtil.getOfficialShiftInfo().currentShift().toString());
      Logger.recordOutput(
          "HubShiftUtil/officialShiftInfo/active", HubShiftUtil.getOfficialShiftInfo().active());
      Logger.recordOutput(
          "HubShiftUtil/officialShiftInfo/elapsedTime",
          HubShiftUtil.getOfficialShiftInfo().elapsedTime());
      Logger.recordOutput(
          "HubShiftUtil/officialShiftInfo/remainingTime",
          HubShiftUtil.getOfficialShiftInfo().remainingTime());

      Logger.recordOutput(
          "HubShiftUtil/shiftedShiftInfo/currentShift",
          HubShiftUtil.getShiftedShiftInfo().currentShift().toString());
      Logger.recordOutput(
          "HubShiftUtil/shiftedShiftInfo/active", HubShiftUtil.getShiftedShiftInfo().active());
      Logger.recordOutput(
          "HubShiftUtil/shiftedShiftInfo/elapsedTime",
          HubShiftUtil.getShiftedShiftInfo().elapsedTime());
      Logger.recordOutput(
          "HubShiftUtil/shiftedShiftInfo/remainingTime",
          HubShiftUtil.getShiftedShiftInfo().remainingTime());
    }
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
