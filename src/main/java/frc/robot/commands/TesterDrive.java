package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class TesterDrive extends Command {
  private Drive drive;
  private String FLStatus;
  private String FRStatus;
  private String BLStatus;
  private String BRStatus;
  private Color FLColorStatus;
  private Color FRColorStatus;
  private Color BLColorStatus;
  private Color BRColorStatus;
  private String testName;
  private String driveConnectionMessage = " Drive Not Connected";

  // If Not working = color
  // If working = green
  // If not connected = red
  // If too slow = orange
  // If pulling too much current = blue;
  // still running Tests = purple
  public TesterDrive(Drive drive, String testName) {
    this.drive = drive;
    this.testName = testName;
  }

  @Override
  public void initialize() {
    FLColorStatus = Constants.NetworkTables.purple;
    FRColorStatus = Constants.NetworkTables.purple;
    BLColorStatus = Constants.NetworkTables.purple;
    BRColorStatus = Constants.NetworkTables.purple;
    FLStatus = "";
    FRStatus = "";
    BLStatus = "";
    BRStatus = "";
    setColorStatus();
    setTextStatus();
    Logger.recordOutput("Tester/Drive", testName);
  }

  @Override
  public void execute() {
    if (!drive.isDriveConnected(0)) {
      FLColorStatus = Constants.NetworkTables.red;
      FLStatus = "FL " + driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(0)) {
      FLColorStatus = Constants.NetworkTables.orange;
      FLStatus =
          "FL "
              + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(0) / drive.requestedSpeed) * 100))
              + "% ";
    } else {
      FLColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isDriveConnected(1)) {
      FRColorStatus = Constants.NetworkTables.red;
      FRStatus = "FR " + driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(1)) {
      FRColorStatus = Constants.NetworkTables.orange;
      FRStatus =
          "FR "
              + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(1) / drive.requestedSpeed) * 100))
              + "%";
    } else {
      FRColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isDriveConnected(2)) {
      BLColorStatus = Constants.NetworkTables.red;
      BLStatus = "BL " + driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(2)) {
      BLColorStatus = Constants.NetworkTables.orange;
      BLStatus =
          "BL "
              + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(2) / drive.requestedSpeed) * 100))
              + "%";

    } else {
      BLColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isDriveConnected(3)) {
      BRColorStatus = Constants.NetworkTables.red;
      BRStatus = driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(3)) {
      BRColorStatus = Constants.NetworkTables.orange;
      BRStatus =
          "BR "
              + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(3) / drive.requestedSpeed) * 100))
              + "%";
    } else {
      BRColorStatus = Constants.NetworkTables.green;
    }

    setColorStatus();

    setTextStatus();
  }

  @Override
  public void end(boolean interrupted) {}

  private void setColorStatus() {
    SmartDashboard.putString(Constants.Tester.DriveColorKeyFL, FLColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.DriveColorKeyFR, FRColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.DriveColorKeyBL, BLColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.DriveColorKeyBR, BRColorStatus.toHexString());
  }

  private void setTextStatus() {
    SmartDashboard.putString(Constants.Tester.DriveKeyFL, FLStatus);
    SmartDashboard.putString(Constants.Tester.DriveKeyFR, FRStatus);
    SmartDashboard.putString(Constants.Tester.DriveKeyBL, BLStatus);
    SmartDashboard.putString(Constants.Tester.DriveKeyBR, BRStatus);
  }
}
